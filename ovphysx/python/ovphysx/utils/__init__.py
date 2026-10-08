# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-UTILS-001
# @covers AC-1 AC-1a AC-2 AC-9 AC-13 AC-15

"""USD physics authoring utilities.

``ovphysx.api.PhysX`` simulates a stage; this subpackage mostly builds one. Its
bulk is a pure-Python layer over ``pxr`` that authors the geometry, xform ops
and physics API schemas a scene needs, so a caller does not have to spell out a
rigid body's collider, mass and transform by hand for every prim. The one
exception is :mod:`~ovphysx.utils.simulation`, described at the end.

The helpers come from ``omni.physx.scripts``, whose names mixed ``camelCase``
and ``snake_case``. Every name here is ``snake_case``, so ``setCollider`` is
:func:`~ovphysx.utils.authoring.set_collider` and
``createAPISchemaPropertyCache`` is
:func:`~ovphysx.utils.schema.create_api_schema_property_cache`. Aliasing the
subpackage on import keeps a ported snippet close to its original shape::

    from ovphysx import utils as physicsUtils
    physicsUtils.add_rigid_box(stage, "/World/box", position=Gf.Vec3f(0, 0, 5))

Eight ``camelCase`` spellings survive as deprecated aliases: ``setCollider``,
``setRigidBody``, ``removeCollider``, ``removePhysics``, ``removeRigidBodySubtree``,
``hasSchema``, ``createJoint`` and ``extractTriangleSurfaceFromTetra``. Each
forwards to the new spelling and raises a ``DeprecationWarning`` naming both, and
each is reachable as ``ovphysx.utils.<name>`` and by explicit import. None is in
``__all__``, so ``from ovphysx.utils import *`` and the rendered documentation
show the new spellings only. No other ``camelCase`` name is kept.

Keyword parameters are ``snake_case`` on the same terms, so
``setCollider(prim, approximationShape=...)`` becomes
``set_collider(prim, approximation_shape=...)``. Three old spellings stay
accepted, being the ones on the helpers above: ``approximationShape`` on
:func:`~ovphysx.utils.authoring.set_collider` and
:func:`~ovphysx.utils.authoring.set_rigid_body`, and ``schemaName`` on
:func:`~ovphysx.utils.schema.has_schema`. Each raises a ``DeprecationWarning``
naming both spellings; passing both in one call raises ``TypeError``.

Importing this subpackage does not load the ovphysx native library, contact a
USD resolver or start a simulation, so it is usable in a process that only
authors USD. Diagnostics go through the standard ``logging`` module under the
``ovphysx.utils`` logger hierarchy.

The authoring helpers need ``pxr``. An authoring process supplies its own stock
``usd-core``, and this subpackage is reached with an explicit
``import ovphysx.utils`` so that ``import ovphysx`` keeps working for
simulation-only users who have no ``pxr``.

``pxr`` is required only once an authoring name is actually touched, not to
import this subpackage. Submodules load on first attribute access, so
``from ovphysx.utils import step_and_write_to_ovstage`` resolves in a process
that has ``ovstage`` but no USD at all -- which is what the ``output_read``
sample does.

ovphysx ships the PhysX schemas *codeless*, so PhysX API schemas are applied by
identifier and their properties authored by name; core ``UsdPhysics`` schemas
still use their typed bindings.
Register the schemas before opening any stage::

    from ovphysx.utils import codeless
    codeless.register_schemas()

Refer to :mod:`~ovphysx.utils.codeless` for the registration rules, which are
strict about ordering and fail silently when ignored.

The submodules group the surface by theme, and every name is also re-exported
here, so both ``ovphysx.utils.shapes.add_rigid_box`` and
``ovphysx.utils.add_rigid_box`` resolve:

- :mod:`~ovphysx.utils.codeless` - registration and access for the codeless
  PhysX schemas
- :mod:`~ovphysx.utils.schema` - schema registry introspection, API schema
  property snapshot and restore
- :mod:`~ovphysx.utils.transform` - xform op stacks, basis vectors, joint-relative
  transforms
- :mod:`~ovphysx.utils.paths` - collision-free stage paths for a prim about to be
  defined
- :mod:`~ovphysx.utils.shapes` - cube, sphere, capsule, cylinder, cone and xform
  constructors, plain, collider and rigid-body flavored
- :mod:`~ovphysx.utils.planes` - ground planes and sized quad planes
- :mod:`~ovphysx.utils.joints` - joints between two prims or between a prim and
  the world
- :mod:`~ovphysx.utils.materials` - physics materials and their binding to prims
- :mod:`~ovphysx.utils.filtering` - collision groups and filtered pairs
- :mod:`~ovphysx.utils.authoring` - mass, force, physics scenes and the collider
  and rigid-body API sets on prims that already exist
- :mod:`~ovphysx.utils.mesh` - procedural meshes, triangle and tetrahedron mesh math
- :mod:`~ovphysx.utils.particles` - particle systems, particle sets, PBD materials
- :mod:`~ovphysx.utils.deformable` - volume and surface deformable bodies and
  their materials
- :mod:`~ovphysx.utils.simulation` - stepping a running simulation and writing
  its output back to an attached ovstage Stage
- :mod:`~ovphysx.utils.constants` - shared tokens and limits

``ovphysx.utils`` is the home for any supported ovphysx helper that does not
belong in the top-level API, a wider remit than USD authoring alone:
:mod:`~ovphysx.utils.simulation` is the one such submodule today.
A submodule of that kind - one that drives a simulation rather than authoring a
stage - must import runtime dependencies such as ``ovstage``, ``warp`` and
``numpy`` inside its functions rather than
at module scope, and reach ``ovphysx.api`` either the same way or under
``if TYPE_CHECKING:``, so that ``import ovphysx.utils`` keeps needing nothing
beyond ``pxr`` and the standard library and still loads no native library. Such
a helper may need ``ovstage`` and a live ``PhysX`` instance when called; only
the import-time guarantee is subpackage-wide.
"""

import importlib as _importlib
import typing as _typing

_SUBMODULES = (
    "simulation",
    "codeless",
    "constants",
    "paths",
    "transform",
    "schema",
    "mesh",
    "shapes",
    "planes",
    "joints",
    "materials",
    "filtering",
    "particles",
    "deformable",
    "authoring",
)

# codeless is not flattened. Its names are generic enough (apply_api, set_attr,
# get_attr) that hoisting them into ovphysx.utils would read as if they were part
# of the authoring surface; reach them through the submodule instead.
_FLATTENED = tuple(name for name in _SUBMODULES if name != "codeless")

# This dependency-free manifest distinguishes a known flat export from an
# unknown attribute without importing a module that needs pxr. The surface test
# checks it against each owning module's __all__ list.
_FLATTENED_EXPORTS = {
    "simulation": ("OvStageOutputCache", "step_and_write_to_ovstage"),
    "constants": (
        "MAX_FLOAT",
        "AXES_INDICES",
        "TOKEN_TRIANGLE_MESH",
        "TOKEN_SDF",
        "TOKEN_SPHERE_FILL",
        "SCENE_UPDATE_TYPE_SYNCHRONOUS",
        "SCENE_UPDATE_TYPE_ASYNCHRONOUS",
        "SCENE_UPDATE_TYPE_DISABLED",
        "COOKED_DATA_TOKENS",
        "MESH_APPROXIMATIONS",
        "METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES",
    ),
    "paths": ("get_stage_next_free_path",),
    "transform": (
        "get_axis_aligned_vector",
        "get_forward_vector",
        "get_basis",
        "get_unit_scale_factor",
        "get_translation",
        "get_world_position",
        "get_aligned_body_transform",
        "set_or_add_scale_op",
        "set_or_add_translate_op",
        "set_or_add_orient_op",
        "set_or_add_scale_orient_translate",
        "setup_transform_as_scale_orient_translate",
        "copy_transform_as_scale_orient_translate",
    ),
    "schema": (
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
    ),
    "mesh": (
        "create_mesh",
        "create_mesh_square_axis",
        "create_mesh_concave",
        "create_mesh_cube",
        "create_mesh_cylinder",
        "create_mesh_cone",
        "compute_bounding_box_diagonal",
        "triangulate_mesh",
        "extract_triangle_surface_from_tetra",
        "create_triangle_mesh_square",
        "calculate_tetra_volume",
        "fixup_tetra_mesh_volumes",
        "verify_tetra_mesh",
        "cube_tetrahedra",
        "create_tetra_voxels",
        "create_tetra_voxel_box",
        "create_tetra_voxel_sphere",
        "create_triangle_mesh_cube",
        "convert_tetra_to_triangle_soup",
    ),
    "shapes": (
        "add_box",
        "add_collider_box",
        "add_rigid_box",
        "add_cube",
        "add_collider_cube",
        "add_rigid_cube",
        "add_sphere",
        "add_collider_sphere",
        "add_rigid_sphere",
        "add_capsule",
        "add_collider_capsule",
        "add_rigid_capsule",
        "add_cylinder",
        "add_collider_cylinder",
        "add_rigid_cylinder",
        "add_cone",
        "add_collider_cone",
        "add_rigid_cone",
        "add_xform",
        "add_rigid_xform",
    ),
    "planes": (
        "add_ground_plane",
        "add_quad_plane",
        "add_cube_ground_plane",
        "add_plane_collider",
    ),
    "joints": (
        "add_joint_fixed",
        "create_joint",
        "create_joints",
    ),
    "materials": (
        "add_physics_material_to_prim",
        "ensure_material_on_path",
        "add_rigid_body_material",
    ),
    "filtering": (
        "add_collision_to_collision_group",
        "remove_collision_from_collision_group",
        "is_in_collision_group",
        "add_collision_group",
        "add_pair_filter",
        "remove_pair_filter",
    ),
    "particles": (
        "create_particles_grid",
        "add_physx_particle_system",
        "add_pbd_particle_material",
        "add_pbd_material_water",
        "add_pbd_material_viscous",
        "add_physx_particleset_points",
        "add_physx_particleset_pointinstancer",
        "configure_particle_set",
        "add_physx_particle_anisotropy",
        "add_physx_particle_smoothing",
        "add_physx_particle_isosurface",
        "add_physx_diffuse_particles",
        "poisson_sample_mesh",
    ),
    "deformable": (
        "set_physics_volume_deformable_body",
        "create_auto_volume_deformable_hierarchy",
        "set_physics_surface_deformable_body",
        "create_auto_surface_deformable_hierarchy",
        "remove_auto_deformable_body",
        "remove_auto_deformable_hexahedral_mesh",
        "add_auto_deformable_mesh_simplification",
        "remove_auto_deformable_mesh_simplification",
        "remove_deformable_body",
        "add_deformable_material",
        "add_surface_deformable_material",
    ),
    "authoring": (
        "add_density",
        "add_mass",
        "add_force_torque",
        "set_physics",
        "set_collider",
        "set_collider_subtree",
        "set_rigid_body",
        "set_static_collider",
        "remove_physics",
        "remove_collider",
        "remove_collider_subtree",
        "remove_rigid_body",
        "remove_rigid_body_subtree",
        "remove_static_collider",
        "add_physics_scene",
        "set_physics_scene_asyncsimrender",
        "set_local_space_velocities",
        "clear_local_space_velocities",
    ),
}
_FLATTENED_EXPORT_OWNERS = {entry: submodule for submodule, entries in _FLATTENED_EXPORTS.items() for entry in entries}
if tuple(_FLATTENED_EXPORTS) != _FLATTENED or len(_FLATTENED_EXPORT_OWNERS) != sum(
    len(entries) for entries in _FLATTENED_EXPORTS.values()
):
    raise RuntimeError("ovphysx.utils flattened export manifest is inconsistent")

# The eight kept camelCase spellings (AC-15), mapped to the submodule that owns
# the renamed helper. They are absent from the flat export manifest and each
# submodule's __all__, which keeps them out of `import *`, __dir__ and the
# rendered docs.
_DEPRECATED_ALIASES = {
    "createJoint": "joints",
    "extractTriangleSurfaceFromTetra": "mesh",
    "hasSchema": "schema",
    "removeCollider": "authoring",
    "removePhysics": "authoring",
    "removeRigidBodySubtree": "authoring",
    "setCollider": "authoring",
    "setRigidBody": "authoring",
}

if _typing.TYPE_CHECKING:
    from . import (
        authoring,
        codeless,
        constants,
        deformable,
        filtering,
        joints,
        materials,
        mesh,
        particles,
        paths,
        planes,
        schema,
        shapes,
        simulation,
        transform,
    )
    from .authoring import *  # noqa: F403
    from .constants import *  # noqa: F403
    from .deformable import *  # noqa: F403
    from .filtering import *  # noqa: F403
    from .joints import *  # noqa: F403
    from .materials import *  # noqa: F403
    from .mesh import *  # noqa: F403
    from .particles import *  # noqa: F403
    from .paths import *  # noqa: F403
    from .planes import *  # noqa: F403
    from .schema import *  # noqa: F403
    from .shapes import *  # noqa: F403
    from .simulation import *  # noqa: F403
    from .transform import *  # noqa: F403


def _load(name):
    module = _importlib.import_module("." + name, __name__)
    globals()[name] = module
    return module


def __getattr__(name):
    """Resolve a submodule or a flattened helper, importing only what it needs."""
    if name.startswith("__") and name.endswith("__") and name != "__all__":
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    if name in _SUBMODULES:
        return _load(name)
    if name in _DEPRECATED_ALIASES:
        value = getattr(_load(_DEPRECATED_ALIASES[name]), name)
        globals()[name] = value
        return value
    if name == "__all__":
        value = [*_SUBMODULES, *(entry for module in _FLATTENED for entry in _load(module).__all__)]
        globals()["__all__"] = value
        return value
    submodule = _FLATTENED_EXPORT_OWNERS.get(name)
    if submodule is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(_load(submodule), name)
    globals()[name] = value
    return value


def __dir__():
    # Tolerant where __all__ is deliberately not: dir() must answer without the
    # authoring surface. A partial `import *` would hide the missing dependency.
    # Only ImportError is skipped, so other failures remain.
    names = set(_SUBMODULES)
    for submodule in _FLATTENED:
        try:
            module = _load(submodule)
        except ImportError:
            continue
        names.update(getattr(module, "__all__", ()))
    return sorted(names)
