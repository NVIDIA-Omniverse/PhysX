# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""The ``camelCase`` names and parameters AC-15 keeps, exercised rather than inspected.

`test_utils_surface.py` asserts what the eight aliases *are* -- that the set is
the measured eight, that each stays out of `__all__`, that `functools.wraps`
carried the signature over. This file asserts what they *do*: each is called for
real, against a stage where the target authors one, and the stage it produces is
compared with the one the renamed helper produces from the same starting point.
An alias wired to the wrong target passes every check in the other file and
fails here.

The three kept parameter spellings are covered on the same terms, and need the
equivalence case for a reason of their own: a shim that warned and then dropped
the value would leave the helper running on its default, which no check on the
warning alone can see.
"""

# @implements REQ-PYTHON-UTILS-001
# @covers AC-15

import warnings
from pathlib import Path

import pytest

pytest.importorskip("pxr", reason="ovphysx.utils requires a caller-supplied USD runtime")

from pxr import Gf, UsdGeom  # noqa: E402

from ovphysx import utils  # noqa: E402

# Each entry drives one alias end to end: `exercise` receives the function under
# test -- alias or renamed helper -- plus a stage to author into, and returns
# whatever the call yields. The test compares both the return value and the
# resulting stage, so a helper that returns None is still covered.
#
# `set_rigid_body`'s `approximation_shape`/`kinematic` are positional in the
# recovered signature, hence the positional calls here.


def _box(stage, path="/World/box"):
    prim = utils.add_box(stage, path, size=Gf.Vec3f(1.0))
    return prim


def _collider_box(stage, path="/World/box"):
    prim = _box(stage, path)
    utils.set_collider(prim, "convexHull")
    return prim


def _rigid_box(stage, path="/World/box"):
    prim = _box(stage, path)
    utils.set_rigid_body(prim, "convexHull", False)
    return prim


def _subtree(stage):
    UsdGeom.Xform.Define(stage, "/World/root")
    utils.set_rigid_body(_box(stage, "/World/root/child"), "convexHull", False)
    return stage.GetPrimAtPath("/World/root")


# A single tetrahedron: four points, one interior-free tet, so every face is a
# surface face.
TETRA_POINTS = [Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 0, 1)]
TETRA_INDICES = [0, 1, 2, 3]

CASES = {
    "setCollider": lambda fn, stage: fn(_box(stage), "convexHull"),
    "setRigidBody": lambda fn, stage: fn(_box(stage), "convexHull", False),
    "removeCollider": lambda fn, stage: fn(_collider_box(stage)),
    "removePhysics": lambda fn, stage: fn(_rigid_box(stage)),
    "removeRigidBodySubtree": lambda fn, stage: fn(_subtree(stage)),
    "hasSchema": lambda fn, stage: fn(_rigid_box(stage), "PhysicsRigidBodyAPI"),
    "createJoint": lambda fn, stage: fn(
        stage, "Fixed", _rigid_box(stage, "/World/a"), _rigid_box(stage, "/World/b")
    ).GetPath(),
    "extractTriangleSurfaceFromTetra": lambda fn, stage: fn(TETRA_POINTS, TETRA_INDICES),
}

# One row per kept parameter spelling: the two spellings, a value to pass, the
# prim to author against, and the rest of the call. `_call` below assembles the
# call from a row, so one row serves the warning case, the equivalence case and
# both of the both-spellings cases.
PARAMETER_CASES = {
    "has_schema": ("schemaName", "schema_name", "PhysicsRigidBodyAPI", _rigid_box, {}),
    "set_collider": ("approximationShape", "approximation_shape", "convexHull", _box, {}),
    "set_rigid_body": (
        "approximationShape",
        "approximation_shape",
        "convexHull",
        _box,
        {"kinematic": False},
    ),
}


def _call(name, stage, *spellings, positional=None):
    """Call `name` with the row's value bound to each of `spellings`.

    `positional` passes that same value positionally as well, which is the only
    way to reach the both-spellings guard on `set_rigid_body`, whose renamed
    parameter a caller may supply either way.
    """
    _, _, value, make_prim, rest = PARAMETER_CASES[name]
    args = (make_prim(stage), value) if positional else (make_prim(stage),)
    kwargs = dict.fromkeys(spellings, value)
    return getattr(utils, name)(*args, **kwargs, **rest)


def _new_spelling(old):
    """The renamed helper an alias forwards to, taken from the alias itself."""
    return getattr(utils, old).__wrapped__.__name__


def test_every_alias_is_exercised():
    """A ninth alias without a case here would otherwise be silently untested."""
    assert set(CASES) == set(utils._DEPRECATED_ALIASES)


@pytest.mark.parametrize("old", sorted(CASES))
def test_alias_warns_once_and_names_both_spellings(old, stage):
    """One `DeprecationWarning` per call, quoting the name to migrate to."""
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        CASES[old](getattr(utils, old), stage)

    deprecations = [w for w in caught if issubclass(w.category, DeprecationWarning)]
    assert len(deprecations) == 1, [str(w.message) for w in deprecations]

    message = str(deprecations[0].message)
    assert old in message
    assert _new_spelling(old) in message


@pytest.mark.parametrize("old", sorted(CASES))
def test_alias_and_renamed_helper_author_the_same_stage(old, codeless_schemas):
    """The forwarding is real: same return value, same resulting stage.

    Two stages rather than one, since several of these mutate the prim they are
    given and would not be idempotent on a second call.
    """
    from pxr import Usd

    results = []
    layers = []
    for fn in (getattr(utils, old), getattr(utils, _new_spelling(old))):
        stage = Usd.Stage.CreateInMemory()
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", DeprecationWarning)
            results.append(CASES[old](fn, stage))
        layers.append(stage.GetRootLayer().ExportToString())

    assert results[0] == results[1]
    assert layers[0] == layers[1]


def test_every_kept_parameter_is_exercised():
    """A fourth shim without a case here would otherwise be silently untested."""
    import ovphysx.utils as utils_module

    shimmed = {
        name
        for name in utils_module.__all__
        if getattr(getattr(utils_module, name), "__deprecated_parameters__", ())
    }
    assert shimmed == set(PARAMETER_CASES)


@pytest.mark.parametrize("name", sorted(PARAMETER_CASES))
def test_camel_parameter_warns_and_names_both_spellings(name, stage):
    """One `DeprecationWarning` per call, quoting the spelling to migrate to."""
    old, new = PARAMETER_CASES[name][:2]

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        _call(name, stage, old)

    deprecations = [w for w in caught if issubclass(w.category, DeprecationWarning)]
    assert len(deprecations) == 1, [str(w.message) for w in deprecations]

    message = str(deprecations[0].message)
    assert old in message
    assert new in message
    assert Path(deprecations[0].filename).resolve() == Path(__file__).resolve()


@pytest.mark.parametrize(
    "name, alias",
    (
        ("has_schema", "hasSchema"),
        ("set_collider", "setCollider"),
        ("set_rigid_body", "setRigidBody"),
    ),
)
def test_camel_parameter_through_alias_warns_at_the_caller(name, alias, stage):
    """Both nested deprecations identify the consumer's file."""
    old, _, value, make_prim, rest = PARAMETER_CASES[name]
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        getattr(utils, alias)(make_prim(stage), **{old: value}, **rest)

    deprecations = [w for w in caught if issubclass(w.category, DeprecationWarning)]
    assert len(deprecations) == 2, [str(w.message) for w in deprecations]
    expected = Path(__file__).resolve()
    assert all(Path(w.filename).resolve() == expected for w in deprecations), [
        w.filename for w in deprecations
    ]


@pytest.mark.parametrize("name", sorted(PARAMETER_CASES))
def test_camel_parameter_reaches_the_helper(name, codeless_schemas):
    """Same answer and same resulting stage under either spelling.

    Two stages rather than one: the two collider helpers mutate the prim they
    are given and refuse a prim that is already a collider.
    """
    from pxr import Usd

    old, new = PARAMETER_CASES[name][:2]

    results = []
    layers = []
    for spelling in (old, new):
        stage = Usd.Stage.CreateInMemory()
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", DeprecationWarning)
            results.append(_call(name, stage, spelling))
        layers.append(stage.GetRootLayer().ExportToString())

    assert results[0] == results[1]
    assert layers[0] == layers[1]


@pytest.mark.parametrize("name", sorted(PARAMETER_CASES))
def test_both_spellings_as_keywords_is_an_error(name, stage):
    """Nothing in such a call says which of the two values was meant."""
    old, new = PARAMETER_CASES[name][:2]

    with pytest.raises(TypeError) as raised:
        _call(name, stage, old, new)

    assert old in str(raised.value)
    assert new in str(raised.value)


@pytest.mark.parametrize("name", sorted(PARAMETER_CASES))
def test_the_camel_spelling_over_a_positional_value_is_an_error(name, stage):
    """The same collision, reached the other way a caller can reach it.

    All three renamed parameters may be supplied positionally. A shim looking
    at the keywords alone still fails the call, but on Python's own duplicate
    argument after having warned -- so the caller is told to migrate to the
    spelling they already used. Hence the absence of a warning is asserted
    here as well as the diagnosis.
    """
    old, new = PARAMETER_CASES[name][:2]

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        with pytest.raises(TypeError) as raised:
            _call(name, stage, old, positional=True)

    assert old in str(raised.value)
    assert new in str(raised.value)
    assert not [w for w in caught if issubclass(w.category, DeprecationWarning)]
