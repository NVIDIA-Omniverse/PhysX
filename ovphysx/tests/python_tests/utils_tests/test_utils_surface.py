# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Shape of the ovphysx.utils public surface and its import discipline."""

# @implements REQ-PYTHON-UTILS-001
# @covers AC-1 AC-2 AC-9 AC-11 AC-13 AC-15

import ast
import importlib.util
import inspect
import itertools
import subprocess
import sys
import warnings
from pathlib import Path

import pytest

pytest.importorskip("pxr", reason="ovphysx.utils requires a caller-supplied USD runtime")

SUBMODULES = (
    "authoring",
    "codeless",
    "constants",
    "deformable",
    "filtering",
    "joints",
    "materials",
    "mesh",
    "particles",
    "paths",
    "planes",
    "schema",
    "shapes",
    "simulation",
    "transform",
)

# The submodules whose whole public surface is flat re-exported. `codeless` is
# the one AC-2 exempts; see CODELESS_ONLY_NAMES.
FLATTENED_SUBMODULES = tuple(name for name in SUBMODULES if name != "codeless")

# Every source file in the package, public submodule or not. The import-discipline
# checks below read files rather than the public surface, so a private module has
# to be named here to be checked at all -- `_deprecation` exports nothing and so
# appears in no other list in this file.
SOURCE_MODULES = SUBMODULES + ("_deprecation", "__init__")

# The eight omni.physx spellings AC-15 keeps alive, with the submodule that owns
# the renamed helper each forwards to.
DEPRECATED_ALIASES = {
    "createJoint": ("joints", "create_joint"),
    "extractTriangleSurfaceFromTetra": ("mesh", "extract_triangle_surface_from_tetra"),
    "hasSchema": ("schema", "has_schema"),
    "removeCollider": ("authoring", "remove_collider"),
    "removePhysics": ("authoring", "remove_physics"),
    "removeRigidBodySubtree": ("authoring", "remove_rigid_body_subtree"),
    "setCollider": ("authoring", "set_collider"),
    "setRigidBody": ("authoring", "set_rigid_body"),
}

# The three `camelCase` parameter spellings AC-15 keeps alive, keyed by the
# helper that accepts each and mapped to the renamed parameter it forwards to.
# The bound of three is enforced here for the same reason the bound of eight is
# enforced on `_DEPRECATED_ALIASES`: a fourth is a policy change. There is no
# table in `utils/__init__.py` to count, because a parameter is resolved inside
# the call rather than by `__getattr__`, so the wrapper carries its own record.
DEPRECATED_PARAMETERS = {
    ("has_schema", "schemaName"): "schema_name",
    ("set_collider", "approximationShape"): "approximation_shape",
    ("set_rigid_body", "approximationShape"): "approximation_shape",
}

# AC-2's headline figures. Every other check of the flat surface derives its
# expectation from the submodules' own `__all__` lists, so a helper deleted from
# a submodule's `__all__` and from the flat surface together fails none of them.
# These two counts are what notices it.
FLAT_HELPER_COUNT = 138
ALL_ENTRY_COUNT = FLAT_HELPER_COUNT + len(SUBMODULES)

# The public surface of `codeless`, which AC-2 requires to be reachable only
# through `ovphysx.utils.codeless`. Spellings this generic would read as part of
# the authoring surface if they were flattened onto `ovphysx.utils`.
CODELESS_ONLY_NAMES = (
    "CodelessSchemaError",
    "apply_api",
    "get_attr",
    "instanced_name",
    "register_schemas",
    "remove_api",
    "schema_is_registered",
    "set_attr",
    "set_attrs",
    "set_rel",
    "try_apply_api",
)

# The 38 public names of `omni.physx.scripts` that the recovered surface must not
# expose, enumerated in full rather than by category so that a reappearance fails
# here (REQ AC-9). Grouped by the module they came from.
EXCLUDED_NAMES = (
    # ifaces.py
    "get_physx_interface",
    "get_physx_simulation_interface",
    "get_physx_cooking_interface",
    "get_physx_cooking_private_interface",
    "get_physx_scene_query_interface",
    "get_physx_attachment_private_interface",
    "get_physx_property_query_interface",
    "get_physx_replicator_interface",
    "get_physx_stage_update_interface",
    "get_physx_statistics_interface",
    "get_physx_visualization_interface",
    "get_physx_benchmarks_interface",
    "get_physxunittests_interface",
    # utils.py
    "new_memory_stage",
    "release_memory_stage",
    "ExpectMessage",
    "safe_import_tests",
    # physicsUtils.py
    "get_initial_collider_pairs",
    # deformableUtils.py
    "compute_conforming_tetrahedral_mesh",
    "compute_voxel_tetrahedral_mesh",
    "create_auto_deformable_attachment",
    # deformableMeshUtils.py
    "loadTetFile",
    # extension.py
    "PhysxExtension",
    # propertyQueryRigidBody.py
    "Query",
    "QueryManager",
    "Result",
    # assets_paths.py
    "AssetFolders",
    "get_server_path",
    "get_asset_path",
    "get_s3_web_path",
    # assets_paths_base.py
    "OV_PATH",
    "S3_BUCKET",
    "S3_PATH",
    "S3_REGION",
    "get_s3_upload_path",
    # pythonUtils.py
    "autoassign",
    "ScopeGuard",
    "get_all_submodules",
)

# The 17 names that were recovered and then dropped on a usefulness re-audit: no
# observable caller, and either no physics content of their own or a dependency
# on a capability ovphysx has no equivalent of (a viewport, a property window, an
# exploded-view render). Pinned by name for the same reason as EXCLUDED_NAMES,
# and kept apart from them because the ground for dropping them is different:
# these were never unimplementable, just not worth a place on the surface.
# Grouped by the `ovphysx.utils` submodule that held each one before the drop,
# which is not the axis the lists either side of it use.
DROPPED_NAMES = (
    # transform.py
    "CameraTransformHelper",
    # constants.py
    "HALF_PI",
    # mesh.py
    "explodeTriangleMesh",
    "explodeTetraMesh",
    "voxel_sphere_test",
    "voxel_pass_all_test",
    "add_tetra",
    # schema.py
    "hasAPI",
    "get_derived_schemas",
    # authoring.py
    "isDefined",
    "get_spatial_tendon_parent_link",
    "get_spatial_tendon_attachment_candidates",
    "has_custom_metadata",
    "get_custom_metadata",
    "set_custom_metadata",
    "clear_custom_metadata",
    # particles.py
    "get_default_particle_system",
)

# The three names that were recovered, are still needed, and are no longer
# public: each has an internal caller, so its body stays in the package under a
# leading-underscore spelling while the public name goes. Each row names the
# surviving implementation as well as the withdrawn spelling, so the case below
# can assert both halves (REQ AC-9).
PRIVATIZED_NAMES = {
    "create_unused_path": ("paths", "_create_unused_path"),
    "add_triangle": ("mesh", "_add_triangle"),
    "get_default_particle_system_path": ("particles", "_get_default_particle_system_path"),
}

# The nine names that had already left `omni.physx.scripts` before the commit
# whose surface the 187-name census measures, so they are in neither list above
# by construction: they were never candidates for recovery, because they no
# longer existed to recover. All nine went on 2026-03-20 in `233417d51a`
# ("OMPE-18178: Remove deprecated deformable schemas, phase I"), all nine carried
# deprecation language in their final source, and every API schema any of them
# applies -- PhysxDeformableAPI, PhysxDeformableBodyAPI,
# PhysxDeformableSurfaceAPI, the two deformable material APIs,
# PhysxParticleClothAPI, PhysxAutoParticleClothAPI and the TetrahedralMesh prim
# type -- is absent from the schemas ovphysx ships, so a recovered version would
# raise from `codeless.apply_api` rather than author anything. Pinned so a later
# recovery pass cannot reintroduce one; see REQ "Superseded before the census"
# for the replacement each maps to. Grouped by the module they were removed from.
SUPERSEDED_NAMES = (
    # deformableUtils.py
    "add_physx_deformable_body",
    "add_physx_deformable_surface",
    "add_deformable_body_material",
    "add_deformable_surface_material",
    "TetMeshData",
    "create_skin_mesh_from_tetrahedral_mesh",
    # particleUtils.py
    "add_physx_particle_cloth",
    "add_physx_particle_cloth_with_constraints",
    "create_spring_grid",
)

# The helpers a dropped name was folded into, plus `has_schema`, the sibling
# `hasAPI` duplicated. A removal that took one of these with it fails here rather
# than at a user's call.
SURVIVORS_OF_DROPPED_NAMES = (
    "add_physics_scene",
    "create_tetra_voxel_box",
    "create_tetra_voxel_sphere",
    "poisson_sample_mesh",
    "set_local_space_velocities",
    "clear_local_space_velocities",
    "has_schema",
)

# Modules that must not be reachable at import time, at any scope. `ovstage` and
# `numpy` are call-time dependencies of the simulation submodule (AC-13); the
# rest were never dependencies at all (AC-1).
BLOCKED_ROOTS = ("carb", "omni", "ovstage", "numpy")


def _utils_dir() -> Path:
    spec = importlib.util.find_spec("ovphysx")
    assert spec is not None and spec.origin is not None, "ovphysx is not importable"
    return Path(spec.origin).resolve().parent / "utils"


# Run in a child process so the blocker cannot be defeated by a module another
# test already imported.
_IMPORT_CHILD = """
import importlib.abc
import sys

# find_spec, not the legacy find_module: Python ignores find_module from 3.12
# on, which would make this blocker silently block nothing.
class _Blocker(importlib.abc.MetaPathFinder):
    def find_spec(self, name, path=None, target=None):
        if name.split(".")[0] in {blocked!r}:
            raise ImportError("blocked for the test: " + name)
        return None

sys.meta_path.insert(0, _Blocker())

# Prove the blocker works before trusting the result of the imports below.
for probe in {blocked!r}:
    try:
        __import__(probe)
    except ImportError:
        pass
    else:
        raise AssertionError("blocker is ineffective for " + probe)

import ovphysx.utils
for name in {submodules!r}:
    __import__("ovphysx.utils." + name)

leaked = [m for m in sys.modules if m.split(".")[0] in {blocked!r}]
assert not leaked, leaked
assert "ovphysx.api" not in sys.modules, "ovphysx.api was imported"
assert not any("_bindings" in m for m in sys.modules), "native bindings imported"
print("OK")
"""


def test_imports_without_kit_ovstage_or_numpy():
    """The whole subpackage imports on `pxr` plus the standard library alone."""
    script = _IMPORT_CHILD.format(blocked=BLOCKED_ROOTS, submodules=SUBMODULES)
    proc = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, timeout=120)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert proc.stdout.startswith("OK")


_TRY_NODES = (ast.Try,) + ((ast.TryStar,) if hasattr(ast, "TryStar") else ())
_NESTED_BODIES = (ast.With, ast.AsyncWith, ast.For, ast.AsyncFor, ast.While)


def _is_type_checking(test) -> bool:
    """Whether an `if` test is the `TYPE_CHECKING` guard, either spelling."""
    if isinstance(test, ast.Name):
        return test.id == "TYPE_CHECKING"
    if isinstance(test, ast.Attribute):
        return test.attr == "TYPE_CHECKING"
    return False


def _import_time_modules(source: str) -> list:
    """Every module a source file imports when it is imported.

    Not the direct children of the module body alone: an import inside a
    top-level `try`, `with`, `if` or loop runs at import time just as a bare one
    does, so a guarded `try: import numpy` would satisfy a shallow scan and
    still load numpy. Descend into those, and skip only the two things that
    genuinely do not run -- a function or class body, and an
    `if TYPE_CHECKING:` branch, whose `else` is still walked because it does.
    """
    imported: list = []

    def visit(body) -> None:
        for node in body:
            if isinstance(node, ast.Import):
                imported.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                if not node.level:
                    imported.append(node.module or "")
            elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                continue
            elif isinstance(node, ast.If):
                if not _is_type_checking(node.test):
                    visit(node.body)
                visit(node.orelse)
            elif isinstance(node, _TRY_NODES):
                visit(node.body)
                for handler in node.handlers:
                    visit(handler.body)
                visit(node.orelse)
                visit(node.finalbody)
            elif isinstance(node, _NESTED_BODIES):
                visit(node.body)
                visit(getattr(node, "orelse", []))

    visit(ast.parse(source).body)
    return imported


@pytest.mark.parametrize("name", SOURCE_MODULES)
def test_no_module_scope_heavy_imports(name):
    """`ovstage` and `numpy` stay inside function bodies; `ovphysx.api` too.

    A module-scope import of a native-free sibling such as `ovphysx.types` is
    allowed, and an `ovphysx.api` import under `if TYPE_CHECKING:` is too, since
    neither runs at import time (REQ AC-13).
    """
    source = (_utils_dir() / f"{name}.py").read_text(encoding="utf-8")

    for module in _import_time_modules(source):
        root = module.split(".")[0]
        assert root not in BLOCKED_ROOTS, f"{name}.py imports {module} at module scope"
        if root == "ovphysx":
            # ovphysx.types is native-free, so importing it costs nothing; the
            # rest of the package, ovphysx.api above all, is not.
            assert module == "ovphysx.types", f"{name}.py imports {module} at module scope"


@pytest.mark.parametrize("name", FLATTENED_SUBMODULES)
def test_flat_reexport_matches_submodule(name):
    """Every public name resolves flat and through its submodule (REQ AC-2)."""
    import ovphysx.utils as utils

    submodule = getattr(utils, name)
    assert name in utils.__all__

    for public in submodule.__all__:
        assert public in utils.__all__, public
        assert getattr(utils, public) is getattr(submodule, public)


def test_flat_export_manifest_matches_its_loaded_owners():
    """The pxr-free lookup manifest must match the actual flat surface."""
    import ovphysx.utils as utils

    actual_owners = {}
    duplicates = []
    exports = {}
    for submodule_name in FLATTENED_SUBMODULES:
        public_names = tuple(getattr(utils, submodule_name).__all__)
        exports[submodule_name] = public_names
        for public_name in public_names:
            if public_name in actual_owners:
                duplicates.append(public_name)
            actual_owners[public_name] = submodule_name

    assert not duplicates, duplicates
    assert utils._FLATTENED_EXPORTS == exports
    assert utils._FLATTENED_EXPORT_OWNERS == actual_owners


def test_codeless_names_are_reachable_only_through_the_submodule():
    """AC-2's one carve-out, asserted positively rather than skipped.

    `codeless` is reached by its own name and its helpers are *not* flattened
    onto `ovphysx.utils`. Both halves matter: a reader who takes AC-2's flat
    promise literally would look for `ovphysx.utils.set_attr`, and a later
    `from .codeless import *` in `utils/__init__.py` would add it silently.
    """
    import ovphysx.utils as utils

    assert "codeless" in utils.__all__
    assert utils.codeless is not None

    assert set(utils.codeless.__all__) == set(CODELESS_ONLY_NAMES), utils.codeless.__all__

    for public in CODELESS_ONLY_NAMES:
        assert hasattr(utils.codeless, public), public
        assert public not in utils.__all__, public
        assert not hasattr(utils, public), public


def test_all_is_resolvable_and_complete():
    import ovphysx.utils as utils

    unresolvable = [name for name in utils.__all__ if not hasattr(utils, name)]
    assert not unresolvable, unresolvable


def test_surface_size_is_pinned():
    """AC-2's 138 flat helpers and 153 `__all__` entries, asserted.

    The counterpart of the length assertions on EXCLUDED_NAMES and
    DROPPED_NAMES: those keep a name from drifting back onto the surface, and
    this keeps one from quietly leaving it. The independent lookup manifest
    checks which names make up that count (REQ AC-2).
    """
    import ovphysx.utils as utils

    duplicates = sorted({name for name in utils.__all__ if utils.__all__.count(name) > 1})
    assert not duplicates, duplicates
    assert len(utils.__all__) == ALL_ENTRY_COUNT, len(utils.__all__)

    assert sorted(name for name in utils.__all__ if name in SUBMODULES) == sorted(SUBMODULES)
    helpers = [name for name in utils.__all__ if name not in SUBMODULES]
    assert len(helpers) == FLAT_HELPER_COUNT, len(helpers)

    # AC-2's other figure: the names the carve-out holds back from that surface.
    assert len(CODELESS_ONLY_NAMES) == 11, len(CODELESS_ONLY_NAMES)


@pytest.mark.parametrize("name", EXCLUDED_NAMES + DROPPED_NAMES + SUPERSEDED_NAMES + tuple(PRIVATIZED_NAMES))
def test_excluded_names_are_absent(name):
    """Nothing needing a removed binding or Kit, nor dropped, nor privatized, nor
    superseded before the census, is exposed (REQ AC-9)."""
    import ovphysx.utils as utils

    assert not hasattr(utils, name), name
    assert name not in utils.__all__, name


def test_excluded_name_list_is_the_whole_recorded_exclusion():
    """The pins cover all 38 + 17 + 3 + 9 names AC-9 enumerates, not a sample.

    Parametrized absence cases cannot notice a row that was deleted from the
    list rather than from the surface, so the counts are asserted directly.
    """
    assert len(set(EXCLUDED_NAMES)) == len(EXCLUDED_NAMES), EXCLUDED_NAMES
    assert len(EXCLUDED_NAMES) == 38, len(EXCLUDED_NAMES)

    assert len(set(DROPPED_NAMES)) == len(DROPPED_NAMES), DROPPED_NAMES
    assert len(DROPPED_NAMES) == 17, len(DROPPED_NAMES)

    assert len(PRIVATIZED_NAMES) == 3, len(PRIVATIZED_NAMES)

    assert len(set(SUPERSEDED_NAMES)) == len(SUPERSEDED_NAMES), SUPERSEDED_NAMES
    assert len(SUPERSEDED_NAMES) == 9, len(SUPERSEDED_NAMES)

    # Pairwise, not just the first pair: a name in two lists has two different
    # recorded grounds and the reader cannot tell which one is the real one.
    pinned = (
        ("excluded", EXCLUDED_NAMES),
        ("dropped", DROPPED_NAMES),
        ("privatized", tuple(PRIVATIZED_NAMES)),
        ("superseded", SUPERSEDED_NAMES),
    )
    for (first, one), (second, other) in itertools.combinations(pinned, 2):
        overlap = set(one) & set(other)
        assert not overlap, (first, second, overlap)


@pytest.mark.parametrize("public", sorted(PRIVATIZED_NAMES))
def test_privatized_names_keep_the_implementation_they_withdrew(public):
    """Each row's private spelling is present, callable and out of `__all__`.

    The absence case above is the other half of what PRIVATIZED_NAMES claims;
    REQ AC-9 states why both halves are needed.
    """
    import ovphysx.utils as utils

    submodule_name, private = PRIVATIZED_NAMES[public]
    submodule = getattr(utils, submodule_name)

    assert callable(getattr(submodule, private, None)), private
    assert private not in submodule.__all__, private
    assert not hasattr(submodule, public), public


@pytest.mark.parametrize("name", SURVIVORS_OF_DROPPED_NAMES)
def test_survivors_of_dropped_names_are_still_present(name):
    """A dropped name's logic was inlined, not deleted along with its caller.

    Six of these called one of DROPPED_NAMES before the re-audit and absorbed it;
    `has_schema` is the sibling `hasAPI` duplicated. Absence cases alone would
    pass just as well if the survivor had been removed too.
    """
    import ovphysx.utils as utils

    assert name in utils.__all__, name
    assert callable(getattr(utils, name)), name


@pytest.mark.parametrize("name", SOURCE_MODULES)
def test_no_kit_import_at_any_scope(name):
    """`carb` and `omni.*` are absent from the sources outright (REQ AC-9).

    Broader than the module-scope check above and than the import-time child:
    a deferred `import carb` inside a function body passes both and would still
    make the helper unusable outside Kit.
    """
    source = (_utils_dir() / f"{name}.py").read_text(encoding="utf-8")

    offenders = []
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            modules = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom):
            modules = [node.module or ""] if not node.level else []
        else:
            continue
        offenders += [m for m in modules if m.split(".")[0] in {"carb", "omni"}]

    assert not offenders, f"{name}.py imports {offenders}"


def test_no_custom_execute_fn_parameter():
    """The Kit undo hook is gone from every set*/remove* helper (REQ AC-11)."""
    import ovphysx.utils as utils

    offenders = []
    for name in utils.__all__:
        obj = getattr(utils, name)
        if not callable(obj) or not (name.startswith("set") or name.startswith("remove")):
            continue
        try:
            params = inspect.signature(obj).parameters
        except (TypeError, ValueError):
            continue
        offenders += [
            (name, forbidden) for forbidden in ("custom_execute_fn", "execute_command_fn") if forbidden in params
        ]
    assert not offenders, offenders


def test_documented_surface_is_snake_case():
    """No camel function spelling survives in `__all__` (REQ AC-15).

    The aliases are the deliberate exception and are asserted absent from
    `__all__` below, so a camel name here is an unrenamed helper.
    """
    import ovphysx.utils as utils

    public_classes = {name for name in utils.__all__ if inspect.isclass(getattr(utils, name))}
    assert public_classes == {"OvStageOutputCache"}
    camel = [
        name
        for name in utils.__all__
        if name not in public_classes and any(char.isupper() for char in name) and not name.isupper()
    ]
    assert not camel, camel


def test_documented_parameters_are_snake_case():
    """No camel spelling survives in a public signature either (REQ AC-15).

    The three kept camel parameters are the deliberate exception and are
    absent from the signatures they are accepted through, since the shim takes
    them out of `**kwargs`; a camel name here is an unrenamed parameter.
    """
    import ovphysx.utils as utils

    offenders = []
    for name in utils.__all__:
        obj = getattr(utils, name)
        if not callable(obj):
            continue
        try:
            params = inspect.signature(obj).parameters
        except (TypeError, ValueError):
            continue
        offenders += [
            (name, parameter)
            for parameter in params
            if any(char.isupper() for char in parameter) and not parameter.isupper()
        ]
    assert not offenders, offenders


def test_alias_set_is_exactly_the_eight_measured_names():
    """AC-15 bounds the aliases; a ninth is a policy change, not a detail."""
    import ovphysx.utils as utils

    assert set(utils._DEPRECATED_ALIASES) == set(DEPRECATED_ALIASES)


def test_parameter_shim_set_is_exactly_the_three_measured_helpers():
    """AC-15 bounds the kept parameters too; a fourth is a policy change.

    Discovered from the surface rather than read out of a table, so a shim
    added to a fourth helper fails here instead of going unrecorded.
    """
    import ovphysx.utils as utils

    found = {
        (name, old): new
        for name in utils.__all__
        for old, new in getattr(getattr(utils, name), "__deprecated_parameters__", ())
    }
    assert found == DEPRECATED_PARAMETERS, found
    assert len(DEPRECATED_PARAMETERS) == 3, len(DEPRECATED_PARAMETERS)


@pytest.mark.parametrize("old", sorted(DEPRECATED_ALIASES))
def test_alias_resolves_but_stays_out_of_the_documented_surface(old):
    """Reachable by attribute and by explicit import, invisible to `import *`.

    `ovphysx.utils.__getattr__` consults its alias table before it searches the
    submodule `__all__` lists, which is what lets a name absent from every
    submodule `__all__` resolve at all; `dir()` and `__all__` are built from `__all__` alone, so the alias
    cannot leak into a star-import or into the rendered docs (REQ AC-15).
    """
    import ovphysx.utils as utils

    submodule_name, new = DEPRECATED_ALIASES[old]

    assert callable(getattr(utils, old))
    assert old not in utils.__all__
    assert old not in dir(utils)
    assert old not in getattr(utils, submodule_name).__all__

    namespace = {}
    exec("from ovphysx.utils import *", namespace)  # noqa: S102 - the point of the test
    assert old not in namespace
    assert new in namespace


@pytest.mark.parametrize("old", sorted(DEPRECATED_ALIASES))
def test_alias_keeps_the_signature_and_docstring_of_its_target(old):
    """`functools.wraps` on the alias, so `help()` answers for either spelling."""
    import ovphysx.utils as utils

    _, new = DEPRECATED_ALIASES[old]
    alias, target = getattr(utils, old), getattr(utils, new)

    assert alias.__wrapped__ is target
    assert inspect.signature(alias) == inspect.signature(target)
    assert alias.__doc__ == target.__doc__


@pytest.mark.parametrize("old", sorted(DEPRECATED_ALIASES))
def test_new_spelling_does_not_warn(old):
    """Only the alias is deprecated; the rename itself is not (REQ AC-15)."""
    import ovphysx.utils as utils

    _, new = DEPRECATED_ALIASES[old]
    with warnings.catch_warnings():
        warnings.simplefilter("error", DeprecationWarning)
        assert callable(getattr(utils, new))
