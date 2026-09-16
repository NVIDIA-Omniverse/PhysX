# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-UTILS-001
# @covers AC-1 AC-10 AC-14

"""Access to ovphysx's *codeless* PhysX USD schemas.

ovphysx ships the PhysX and Omni deformable schemas as codeless artifacts: a
``plugInfo.json`` and ``generatedSchema.usda`` per module. API schemas are
applied by their *schema identifier* and their properties are reached by name::

    from ovphysx.utils import codeless

    codeless.apply_api(prim, "PhysxRigidBodyAPI")
    codeless.set_attr(prim, "physxRigidBody:disableGravity", True)

Multiple-apply schemas take an instance name, which becomes the middle
component of the property path::

    codeless.apply_api(prim, "PhysxCookedDataAPI", "convexHull")
    codeless.set_attr(prim, codeless.instanced_name("physxCookedData", "convexHull", "buffer"), buf)

Registration comes first
^^^^^^^^^^^^^^^^^^^^^^^^
The schemas must be registered with the USD runtime *before* anything in the
process opens a stage or queries the schema registry. USD builds that registry
once, on first access, and never rebuilds it, so a late registration is
silently ineffective and the process it happened in cannot be repaired: there
is no reload, no reset, and a second :func:`register_schemas` call reports
success while changing nothing. Getting the order wrong costs a restart.

Under ovphysx's own bundled runtime, ``import ovphysx`` handles this by
appending to ``OV_PXR_PLUGINPATH_2511``. Under a stock ``usd-core`` -- which is
what ``pxr`` resolves to in an authoring process -- call :func:`register_schemas`
as the first thing in the program, or preset ``PXR_PLUGINPATH_NAME`` before the
process starts. Presetting the environment variable is the only route that also
works inside a host that has already initialized USD, because USD reads it
while *constructing* the registry rather than afterwards. Refer to
``docs/physics_schemas.md``.

The helpers here turn USD's "invalid schema" error into one that names the
likely cause.
"""

import logging
import typing

from pxr import Tf, Usd

logger = logging.getLogger(__name__)

__all__ = [
    "CodelessSchemaError",
    "register_schemas",
    "schema_is_registered",
    "apply_api",
    "try_apply_api",
    "remove_api",
    "instanced_name",
    "get_attr",
    "set_attr",
    "set_attrs",
    "set_rel",
]

# One always-present applied API per codeless schema root ovphysx ships. The
# roots are separate USD plugins and register independently, so probing only the
# PhysX one would report success while the Omni deformable schemas -- which the
# deformable helpers author against -- were missing.
_SENTINEL_IDENTIFIERS = (
    "PhysxRigidBodyAPI",
    "OmniPhysicsDeformableBodyAPI",
)

# Every site that reaches this hint is past saving: the schema registry is
# already built, either because the caller holds a prim and so a stage exists,
# or because register_schemas() just verified that it was beaten to it. Say so,
# rather than offering a call that would report success and change nothing.
_REGISTRATION_HINT = (
    "The ovphysx codeless PhysX schemas are not in USD's schema registry. USD "
    "builds that registry once, on first access, and never rebuilds it, so this "
    "process cannot be repaired: calling "
    "ovphysx.utils.codeless.register_schemas() now would report success and "
    "register nothing usable. Fix the ordering and start again, either by "
    "calling register_schemas() before anything in the process opens a stage or "
    "touches the schema registry, or by presetting PXR_PLUGINPATH_NAME to "
    "os.pathsep.join(str(p) for p in ovphysx.codeless_schema_paths()) before the "
    "process starts. Preset the environment variable if USD may already be "
    "initialized by a host or another library, since that is the only route that "
    "does not depend on running first."
)
_INVALID_PRIM_HINT = "The prim is not valid on this stage."


class CodelessSchemaError(RuntimeError):
    """A codeless PhysX schema could not be applied, removed or authored."""


def register_schemas(verify: bool = False):
    """Register ovphysx's codeless PhysX schemas with the active USD runtime.

    Intended for authoring processes running on a stock ``usd-core``. Call it
    before opening any stage or querying the schema registry; USD builds that
    registry once and a later call cannot repair it, in this process or by any
    other means short of a restart.

    A late call gives no sign of it on its own. ``RegisterPlugins`` still
    reports the plugin roots as registered and ``Tf.Type.FindByName`` still
    resolves the schema types; only applying an API fails, arbitrarily far
    away. Pass ``verify`` to convert that into an immediate error.

    Idempotent: USD ignores a plugin root it has already seen.

    Args:
        verify: Confirm the schemas reached the schema registry and raise if
            they did not. Off by default because the check is not free: the
            only way to ask is to query the schema registry, which builds it,
            so a verified call has to be the last plugin registration in the
            process. Leave it off where another USD component registers
            plugins after ovphysx.

    Returns:
        The list of registered ``Plug.Plugin`` objects, which is empty when the
        schemas were already registered.

    Raises:
        FileNotFoundError: If the staged codeless schema tree cannot be found.
            See :func:`ovphysx.codeless_schema_paths`.
        CodelessSchemaError: If ``verify`` is set and any of the schema roots did
            not reach the schema registry, because something built it first or
            because that root is not staged.
    """
    from pxr import Plug

    from .. import codeless_schema_paths

    registry = Plug.Registry()
    registered = []
    for path in codeless_schema_paths():
        registered.extend(registry.RegisterPlugins(str(path)))
    if verify:
        # Only under `verify`: the probe builds the schema registry, which is
        # the very thing the default path exists to avoid.
        missing = _unregistered_sentinels()
        if missing:
            raise CodelessSchemaError(
                f"register_schemas() registered {len(registered)} plugin root(s), but "
                f"{', '.join(missing)} did not reach the schema registry. Either the "
                "registration was too late, or the schema root declaring it is not "
                f"staged. {_REGISTRATION_HINT}"
            )
    return registered


def schema_is_registered() -> bool:
    """Whether the codeless PhysX schemas are visible to the schema registry.

    Every schema root ovphysx ships has to be there: a partial registration --
    the PhysX schemas without the Omni deformable ones, say -- answers False.

    Note that calling this *builds* the schema registry if it does not exist
    yet, which locks out every later schema plugin registration in the process,
    ovphysx's and any other library's alike. Register first, then query.
    """
    return not _unregistered_sentinels()


def _identifier_is_known(identifier) -> bool:
    """Whether the schema registry knows this applied API schema identifier.

    Answers for the one identifier rather than for the schema set, which is what
    tells a misspelled or unregistered identifier apart from an authoring
    operation USD refused on a schema it knows.
    """
    return bool(Usd.SchemaRegistry().FindAppliedAPIPrimDefinition(identifier))


def _unregistered_sentinels() -> typing.List[str]:
    """The sentinel identifiers the schema registry does not know.

    One query per root rather than one per root's every schema: the registry is
    built by the first of them either way, so this is no more destructive than a
    single probe.
    """
    return [
        identifier for identifier in _SENTINEL_IDENTIFIERS if not _identifier_is_known(identifier)
    ]


def _describe(identifier, instance_name):
    return identifier if instance_name is None else f"{identifier}:{instance_name}"


def _diagnose(identifier):
    """Explain a USD "invalid schema" error for an identifier the registry rejects.

    An unregistered schema set and a misspelled identifier surface as the same
    error, so ask the registry which it was.
    """
    if schema_is_registered():
        return (
            f"'{identifier}' does not name a known API schema. Check the "
            "spelling against the identifiers in "
            "schemas/physx/source/physxSchema/generatedSchema.usda."
        )
    return _REGISTRATION_HINT


def _registration_suffix():
    """The registration hint, for a message that already named its own cause.

    Unlike :func:`_diagnose` this appends rather than replaces, because a
    missing property has a likely cause of its own worth stating first.
    """
    return "" if schema_is_registered() else f" {_REGISTRATION_HINT}"


def _require_concrete_prim_definition(identifier: str) -> None:
    """Raise if ``identifier`` is not a registered concrete prim type."""
    if Usd.SchemaRegistry().FindConcretePrimDefinition(identifier):
        return
    if schema_is_registered():
        reason = f"'{identifier}' does not name a known concrete prim type."
    else:
        reason = _REGISTRATION_HINT
    raise CodelessSchemaError(f"Could not resolve concrete prim type '{identifier}'. {reason}")


def _unresolvable(identifier, instance_name):
    """Why USD cannot resolve this identifier and instance name, if it cannot.

    Four states raise rather than answer, and the registry reports all four. The
    identifier names nothing; or it names a multiple-apply schema and no instance
    name came with it, or one USD does not allow, the empty string for instance;
    or it names a single-apply schema and an instance name came with it. Each is
    a mistake in the call, which no bool describes.

    Returns:
        The cause, for a message, or ``None`` when the pair resolves.
    """
    registry = Usd.SchemaRegistry()
    if not registry.FindAppliedAPIPrimDefinition(identifier):
        return _diagnose(identifier)
    if registry.IsMultipleApplyAPISchema(identifier):
        if instance_name is None:
            return f"'{identifier}' is a multiple-apply API schema and needs an instance name."
        if not Usd.SchemaRegistry.IsAllowedAPISchemaInstanceName(identifier, instance_name):
            return f"'{instance_name}' is not an instance name '{identifier}' allows."
    elif instance_name is not None:
        return (
            f"'{identifier}' is a single-apply API schema and takes no instance "
            f"name, but '{instance_name}' was given."
        )
    return None


def _require_valid_prim(prim, action) -> None:
    """Raise before a codeless operation calls a method on an invalid prim."""
    if not prim:
        raise CodelessSchemaError(f"Could not {action}. {_INVALID_PRIM_HINT}")


def _refusal_hint(prim):
    """Name the edit target's layer when that layer is what refused the write.

    A layer that cannot be edited is the one cause a caller can act on, and it
    is checkable rather than guessable. Any other cause is USD's own, and the
    chained exception carries it.
    """
    if not prim:
        return _INVALID_PRIM_HINT
    stage = prim.GetStage()
    if stage is None:
        return _INVALID_PRIM_HINT
    layer = stage.GetEditTarget().GetLayer()
    if not layer.permissionToEdit:
        return f"The edit target layer '{layer.identifier}' is not editable."
    return "USD refused the operation."


def _edit_schema(action, identifier, instance_name, edit) -> bool:
    """Apply or remove one API schema, telling a refusal from an unresolvable call.

    USD reports two unrelated things the same way: a call it cannot resolve, and
    an operation it resolved and then declined to author -- on a layer that is
    not editable, for instance. Only the registry tells them apart. A call that
    resolves is a refusal, which the caller reports as ``False``; a call that
    does not is a mistake, which raises.

    The registry is asked before the edit, not after USD reports on it. Which of
    ``False`` and ``Tf.ErrorException`` USD answers a mistake with is not a
    property of the mistake, so an answer taken from USD would make the same
    call raise or return by turns. Asking first costs two registry lookups,
    which the prim's own stage has already paid for by existing.

    Args:
        action:        What was attempted, for the message. For example
                       ``"apply PhysxRigidBodyAPI to /World/Cube"``.
        identifier:    The schema identifier the operation names.
        instance_name: The instance name it comes with, or ``None``.
        edit:          A callable performing the edit and returning USD's bool.

    Returns:
        Whether USD authored the edit.

    Raises:
        CodelessSchemaError: If USD cannot resolve the call at all.
    """
    unresolvable = _unresolvable(identifier, instance_name)
    if unresolvable is not None:
        raise CodelessSchemaError(f"Could not {action}. {unresolvable}")
    try:
        return edit()
    except Tf.ErrorException as exc:
        # USD's own text names the layer and the field, which no caller-facing
        # message here carries. Keep it reachable for a caller debugging a refusal.
        logger.debug("USD refused to %s: %s", action, exc)
        return False


def _author(action, prim, write) -> None:
    """Run one property write, raising ``CodelessSchemaError`` if USD refuses it.

    A refused write reaches this two ways -- a ``False`` return and a raised
    ``Tf.ErrorException`` -- and both mean the property was not authored.

    Args:
        action: What was attempted, for the message.
        prim:   The prim being authored, for the message.
        write:  A callable performing the write and returning USD's bool.

    Raises:
        CodelessSchemaError: If the write did not happen.
    """
    try:
        authored = write()
    except Tf.ErrorException as exc:
        raise CodelessSchemaError(f"Could not {action}. {_refusal_hint(prim)}") from exc
    if not authored:
        raise CodelessSchemaError(f"Could not {action}. {_refusal_hint(prim)}")


def apply_api(prim: Usd.Prim, identifier: str, instance_name: typing.Optional[str] = None) -> None:
    """Apply a codeless API schema to a prim by schema identifier.

    Args:
        prim:          The prim to apply the API to.
        identifier:    The schema identifier, e.g. ``"PhysxRigidBodyAPI"``.
        instance_name: The instance name for a multiple-apply schema, e.g.
                       ``"convexHull"`` for ``PhysxCookedDataAPI``. Omit for
                       single-apply schemas.

    Raises:
        CodelessSchemaError: If the API could not be applied. The message
            distinguishes the two usual causes -- the schemas not being
            registered, or the identifier not naming a known API schema -- from
            USD refusing to author a schema it knows.
    """
    description = _describe(identifier, instance_name)
    if not try_apply_api(prim, identifier, instance_name):
        raise CodelessSchemaError(
            f"Could not apply {description} to {prim.GetPath()}. {_refusal_hint(prim)}"
        )


def try_apply_api(
    prim: Usd.Prim, identifier: str, instance_name: typing.Optional[str] = None
) -> bool:
    """Apply a codeless API schema, returning whether USD accepted it.

    The bool-returning half of :func:`apply_api`, for a caller that already
    reports a refused application its own way -- the deformable helpers warn and
    return False rather than raising.

    A refusal is USD declining to author a schema it knows, as for a prim on a
    layer that is not editable. USD reports it either by answering ``False`` or
    by raising, and both come back as ``False`` here. The two causes below are
    not refusals: USD cannot resolve the identifier at all, so it has nothing to
    answer about.

    Args:
        prim:          The prim to apply the API to.
        identifier:    The schema identifier, e.g. ``"PhysxRigidBodyAPI"``.
        instance_name: The instance name for a multiple-apply schema.

    Raises:
        CodelessSchemaError: If the prim is invalid or USD cannot resolve the
            call at all -- the schemas not being registered, the identifier not
            naming a known API schema, or the instance name contradicting the
            one it does name.
    """
    description = _describe(identifier, instance_name)
    _require_valid_prim(prim, f"apply {description}")
    return _edit_schema(
        f"apply {description} to {prim.GetPath()}",
        identifier,
        instance_name,
        lambda: (
            prim.ApplyAPI(identifier)
            if instance_name is None
            else prim.ApplyAPI(identifier, instance_name)
        ),
    )


def remove_api(prim: Usd.Prim, identifier: str, instance_name: typing.Optional[str] = None) -> bool:
    """Remove a codeless API schema from a prim by schema identifier.

    With the schemas registered, removing an API that was never applied is not
    an error; USD reports it as a no-op and this returns its result unchanged,
    so a whole family of mutually exclusive APIs can be stripped without asking
    which one is there.

    The returned bool is USD's answer to whether it authored the removal on the
    current edit target, not a promise that the API is gone: an opinion on a
    stronger layer survives a successful removal on a weaker one. A removal USD
    refuses on a schema it knows -- on a layer that is not editable, for
    instance -- is ``False`` too, on the same terms as :func:`try_apply_api`.

    Args:
        prim:          The prim to strip.
        identifier:    The schema identifier.
        instance_name: The instance name for a multiple-apply schema.

    Returns:
        Whether USD reports the removal as successful.

    Raises:
        CodelessSchemaError: If the prim is invalid or USD cannot resolve the
            call at all, on the same terms as :func:`try_apply_api`.
    """
    description = _describe(identifier, instance_name)
    _require_valid_prim(prim, f"remove {description}")
    return _edit_schema(
        f"remove {description} from {prim.GetPath()}",
        identifier,
        instance_name,
        lambda: (
            prim.RemoveAPI(identifier)
            if instance_name is None
            else prim.RemoveAPI(identifier, instance_name)
        ),
    )


def instanced_name(prefix: str, instance_name: str, property_name: str) -> str:
    """Build the property name of a multiple-apply schema instance.

    For example ``instanced_name("physxCookedData", "convexHull", "buffer")``
    gives ``"physxCookedData:convexHull:buffer"``.
    """
    return f"{prefix}:{instance_name}:{property_name}"


def get_attr(prim: Usd.Prim, name: str) -> Usd.Attribute:
    """Return a prim attribute by name, with a readable error when it is missing.

    Args:
        prim: The prim to read from.
        name: The full property name, e.g. ``"physxRigidBody:disableGravity"``.

    Raises:
        CodelessSchemaError: If the prim is invalid or has no such attribute.
            Usually the owning API was not applied first, or the schemas are
            unregistered.
    """
    _require_valid_prim(prim, f"get attribute '{name}'")
    attr = prim.GetAttribute(name)
    if not attr:
        raise CodelessSchemaError(
            f"{prim.GetPath()} has no attribute '{name}'. Apply the owning API "
            f"schema before authoring its properties.{_registration_suffix()}"
        )
    return attr


def set_attr(prim: Usd.Prim, name: str, value) -> Usd.Attribute:
    """Author a prim attribute by name.

    The attribute must already be declared, which for a codeless schema means
    its API was applied first. This is the codeless replacement for the typed
    ``api.CreateFooAttr(value)`` calls.

    Args:
        prim:  The prim to author on.
        name:  The full property name.
        value: The value to set.

    Returns:
        The authored attribute.

    Raises:
        CodelessSchemaError: If the attribute does not exist or cannot be set.
    """
    attr = get_attr(prim, name)
    _author(f"set '{name}' on {prim.GetPath()}", prim, lambda: attr.Set(value))
    return attr


def set_attrs(prim: Usd.Prim, values: dict) -> None:
    """Author several attributes, skipping the ones whose value is ``None``.

    The authoring helpers expose every schema attribute as an optional keyword
    defaulting to ``None``, so that an unsupplied attribute keeps its schema
    fallback instead of being pinned to a value chosen by the helper. This
    applies that convention in one call.

    Args:
        prim:   The prim to author on.
        values: A mapping of full property name to value. ``None`` values are
                skipped.

    Raises:
        CodelessSchemaError: If the prim is invalid or an attribute cannot be
            set.
    """
    _require_valid_prim(prim, "set attributes")
    for name, value in values.items():
        if value is not None:
            set_attr(prim, name, value)


def set_rel(prim: Usd.Prim, name: str, targets) -> Usd.Relationship:
    """Set a relationship's targets by name.

    Args:
        prim:    The prim to author on.
        name:    The full relationship name, e.g. ``"physxParticle:particleSystem"``.
        targets: A single target path or a list of them.

    Raises:
        CodelessSchemaError: If the prim has no such relationship, or the
            targets could not be set.
    """
    _require_valid_prim(prim, f"set relationship '{name}'")
    path = prim.GetPath()
    rel = prim.GetRelationship(name)
    if not rel:
        raise CodelessSchemaError(
            f"{path} has no relationship '{name}'. Apply the owning "
            f"API schema before authoring its properties.{_registration_suffix()}"
        )
    if not isinstance(targets, (list, tuple)):
        targets = [targets]
    targets = list(targets)
    _author(
        f"set the targets of '{name}' on {path}",
        prim,
        lambda: rel.SetTargets(targets),
    )
    return rel
