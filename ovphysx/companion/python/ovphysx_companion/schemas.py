# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-NATIVE-SCHEMA-001
# @covers AC-1 AC-2
# @implements REQ-COMP-OVSTAGE-001
# @covers AC-2
"""Separate registration of codeless schemas in customer USD and population.

Importing this module loads neither USD image. Call only the registration method
for the image owned by that process, before its first schema lookup/population.
Register all schema bundles together: USD cannot refresh a built schema registry.
"""
from dataclasses import dataclass
from importlib import metadata
from pathlib import Path

# Reserved for StageFlow output. Custom-physics inputs also use companion: names
# (for example companion:stiffness), so only this subnamespace is output-only.
OUTPUT_NAMESPACE = 'companion:output:'


def physx_schema_paths() -> list[Path]:
    """Locate and validate separately installed PhysX schemas without loading USD."""
    try:
        package = metadata.distribution('physx-usd-schemas')
    except metadata.PackageNotFoundError as error:
        raise FileNotFoundError(
            'PhysX schemas are not installed. Install physx-usd-schemas before registration.'
        ) from error
    paths = [Path(package.locate_file(f'physx_usd_schemas/{module}/resources'))
             for module in ('PhysxSchema', 'OmniUsdPhysicsDeformableSchema')]
    for path in paths:
        for name in ('plugInfo.json', 'generatedSchema.usda'):
            if not (path / name).is_file():
                raise FileNotFoundError(
                    f'Incomplete physx-usd-schemas: missing {path / name}. Reinstall the package.'
                )
    return paths


@dataclass(frozen=True)
class ExternalSchema:
    identifier: str
    kind: str
    parameters: tuple

    def __post_init__(self):
        if (not isinstance(self.identifier, str) or not self.identifier or '\0' in self.identifier
                or self.kind not in ('joint', 'geometry')):
            raise ValueError('External schema requires an identifier and joint/geometry kind')
        if isinstance(self.parameters, str):
            raise TypeError('parameters must be a sequence of property names')
        parameters = tuple(self.parameters)
        if not parameters or any(not isinstance(p,str) or not p or '\0' in p for p in parameters) or len(set(parameters)) != len(parameters):
            raise ValueError('Declare distinct, nonempty parameter property names')
        if any(p.startswith(OUTPUT_NAMESPACE) for p in parameters):
            raise ValueError(f'{OUTPUT_NAMESPACE} is reserved for published output and cannot be a solver input')
        object.__setattr__(self, 'parameters', parameters)


@dataclass(frozen=True)
class SchemaBundle:
    plugin: object
    schemas: tuple

    def __post_init__(self):
        path = Path(self.plugin).resolve()
        if path.is_dir():
            path = path / 'plugInfo.json'
        if not path.is_file():
            raise FileNotFoundError(f'Missing codeless plugin descriptor: {path}')
        schemas = tuple(self.schemas)
        if not schemas or any(not isinstance(s, ExternalSchema) for s in schemas):
            raise ValueError('Declare at least one ExternalSchema')
        if len({s.identifier for s in schemas}) != len(schemas):
            raise ValueError('Duplicate schema identifier')
        object.__setattr__(self, 'plugin', str(path))
        object.__setattr__(self, 'schemas', schemas)


def _bundles(bundles, include_core):
    bundles = tuple(bundles)
    if any(not isinstance(bundle, SchemaBundle) for bundle in bundles):
        raise TypeError('Expected SchemaBundle records')
    schemas = tuple(schema for bundle in bundles for schema in bundle.schemas)
    if len({s.identifier for s in schemas}) != len(schemas):
        raise ValueError('Duplicate schema identifier across bundles')
    paths = [bundle.plugin for bundle in bundles]
    if include_core:
        paths[:0] = [str(p) for p in physx_schema_paths()]
    return paths, schemas


def register_customer_schemas(bundles, *, include_core=True):
    """Register and verify in customer-installed pxr, before any stage is opened.

    This final verification initializes USD's registry. A late or malformed
    registration fails explicitly; restart with registration before first use.
    It does not register the separate USD image used by ovstage population.
    """
    paths, schemas = _bundles(bundles, include_core)
    from pxr import Plug, Usd, UsdPhysics
    for path in paths:
        Plug.Registry().RegisterPlugins(path)
    registry = Usd.SchemaRegistry()
    for schema in schemas:
        if schema.kind == 'joint':
            definition = registry.FindConcretePrimDefinition(schema.identifier)
            valid_type = registry.GetTypeFromSchemaTypeName(schema.identifier).IsA(UsdPhysics.Joint._GetStaticTfType())
        else:
            definition = registry.FindAppliedAPIPrimDefinition(schema.identifier)
            valid_type = bool(definition)
        if not definition or not valid_type:
            raise RuntimeError(f'Schema {schema.identifier} is unavailable or has the wrong base; register before USD use and restart')
        missing = set(schema.parameters)-set(map(str,definition.GetPropertyNames()))
        if missing:
            raise ValueError(f'Schema {schema.identifier} does not declare parameters: {sorted(missing)}')
    return tuple(s.identifier for s in schemas)


def register_population_schemas(bundles, *, include_core=True):
    """Register resources separately in ovstage's population USD runtime.

    Requires an ovstage build with schema registration. Resource/schema validity
    is verified by the customer registration and by subsequent population, not
    inferred from success of this path-registration operation.
    """
    paths, schemas = _bundles(bundles, include_core)
    import ovstage
    ovstage.population.register_usd_schemas(paths)
    return tuple(s.identifier for s in schemas)


# @implements REQ-COMP-OVSTAGE-001
# @covers AC-1
def population_description(bundles):
    """Populate resolved world transforms and explicitly select plugin parameters.

    Use with ovstage.population.open_usd_with_desc (or the string counterpart).
    ALL resolves nested transforms in OVStage 0.3; the PHYSICS domain alone
    need not include external parameter columns or composed world transforms.
    """
    _, schemas = _bundles(bundles, False)
    import ovstage
    from ovstage.population import Desc, Selector, PrimPredicate, PropertyPredicate
    selectors = []
    for schema in schemas:
        prim = (PrimPredicate.is_a_type(schema.identifier) if schema.kind == 'joint'
                else PrimPredicate.has_applied_schema(schema.identifier))
        selectors.append(Selector(prim, PropertyPredicate.has_name(*schema.parameters)))
        if schema.kind == 'joint':
            # The coarse domain recognizes built-in concrete joint types. A new
            # derived type also needs inherited joint properties/relationships.
            selectors.append(Selector(prim, PropertyPredicate.has_name(
                'physics:body0', 'physics:body1', 'physics:localPos0', 'physics:localPos1',
                'physics:localRot0', 'physics:localRot1', 'physics:jointEnabled',
                'physics:collisionEnabled', 'physics:excludeFromArticulation',
                'physics:breakForce', 'physics:breakTorque', 'physics:simulationOwner')))
    return Desc(domains=ovstage.PopulationDomain.ALL, selectors=selectors)
