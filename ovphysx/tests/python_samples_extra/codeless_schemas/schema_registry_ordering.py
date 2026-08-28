# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Regression coverage for codeless schema registration ordering (NVBug 6530141).

USD builds its *schema* registry lazily on first access and never rebuilds it.
``Plug.Registry().RegisterPlugins()`` populates the *plugin* registry, a
different structure, and nothing propagates from there into an already-built
schema registry. So the ``register_codeless_schemas.py`` recipe only works in a
process where nothing has touched USD yet -- which is not the case inside a DCC
host, where any USD import/export or any other addon builds the registry first.

The failure is silent: the registered-plugin count and ``Tf.Type.FindByName``
both still report success, and the plugin reports ``isLoaded=True``. Only
``ApplyAPI`` fails. That is why ``register_codeless_schemas.py`` cannot catch
this on its own -- it runs in a fresh process and only ever exercises the
passing case.

This script pins both halves of the documented contract, each arm in a fresh
subprocess:

* ``RegisterPlugins()`` works only when it precedes the first registry access,
  and silently does not when it follows one.
* Presetting ``PXR_PLUGINPATH_NAME`` before the process starts works
  regardless, because USD reads it while *constructing* the registry.

Refer to the ordering warning in ``docs/physics_schemas.md``.
"""

import json
import os
import subprocess
import sys

import ovphysx

# Fallback declared for this attribute in physxSchema's generatedSchema.usda.
# Reading it back on a prim that never authored it proves the schema was
# genuinely resolved rather than merely name-accepted. Update this if the
# schema's declared default ever changes.
SOLVER_POSITION_ITERATION_COUNT_FALLBACK = 16
FALLBACK_ATTRIBUTE = "physxRigidBody:solverPositionIterationCount"


# ---------------------------------------------------------------------------
# Child: one arm, one fresh process
# ---------------------------------------------------------------------------


def run_arm(pre_touch: str, register: bool) -> int:
    """Perform one arm and print its outcome as JSON. Always exits 0.

    ``pre_touch`` selects what accesses USD before registration: ``none``,
    ``registry`` (a schema-registry query) or ``stage`` (an in-memory stage,
    which is what a host's USD import/export amounts to).
    """
    from pxr import Plug, Tf, Usd

    # Hold every stage alive; an expired prim would be a bug in this test
    # rather than the behavior under study.
    keep = []
    if pre_touch == "stage":
        stage = Usd.Stage.CreateInMemory()
        keep.append(stage)
        stage.DefinePrim("/pre", "Xform")
    elif pre_touch == "registry":
        Usd.SchemaRegistry().FindAppliedAPIPrimDefinition("PhysxRigidBodyAPI")

    registered = 0
    if register:
        registry = Plug.Registry()
        for path in ovphysx.codeless_schema_paths():
            registered += len(registry.RegisterPlugins(str(path)))

    type_found = Tf.Type.FindByName("PhysxSchemaPhysxRigidBodyAPI") != Tf.Type.Unknown

    stage = Usd.Stage.CreateInMemory()
    keep.append(stage)
    prim = stage.DefinePrim("/World/Box", "Cube")
    try:
        applied, error = bool(prim.ApplyAPI("PhysxRigidBodyAPI")), ""
    except BaseException as exc:  # Tf.ErrorException, and anything else USD raises
        applied, error = False, str(exc).splitlines()[-1].strip()

    fallback = None
    if applied:
        attribute = prim.GetAttribute(FALLBACK_ATTRIBUTE)
        fallback = attribute.Get() if attribute else None

    print(
        "RESULT "
        + json.dumps(
            {
                "pre_touch": pre_touch,
                "register": register,
                "registered": registered,
                "type_found": type_found,
                "applied": applied,
                "fallback": fallback,
                "error": error,
            }
        )
    )
    return 0


# ---------------------------------------------------------------------------
# Parent: drive the arms and assert
# ---------------------------------------------------------------------------


def _arm(pre_touch: str, register: bool, preset_plugin_path: bool) -> dict:
    """Run one arm in a fresh process and return its parsed outcome."""
    env = os.environ.copy()
    # Setting this in the child's environment is the point: USD reads
    # PXR_PLUGINPATH_NAME while constructing the registry, so it has to be in
    # place before the process starts. Setting it from inside is too late.
    if preset_plugin_path:
        env["PXR_PLUGINPATH_NAME"] = os.pathsep.join(
            str(path) for path in ovphysx.codeless_schema_paths()
        )
    else:
        env.pop("PXR_PLUGINPATH_NAME", None)

    command = [sys.executable, os.path.abspath(__file__), "--arm", pre_touch]
    command.append("--register" if register else "--no-register")
    # Each arm is a bare interpreter plus a usd-core import, so the whole run
    # stays well inside the sample runner's own budget.
    completed = subprocess.run(
        command, env=env, capture_output=True, text=True, timeout=60
    )
    if completed.returncode != 0:
        raise AssertionError(
            f"arm pre_touch={pre_touch} register={register} exited "
            f"{completed.returncode}:\n{completed.stdout}{completed.stderr}"
        )

    for line in completed.stdout.splitlines():
        if line.startswith("RESULT "):
            result = json.loads(line[len("RESULT ") :])
            label = (
                f"pre_touch={pre_touch} register={register} "
                f"PXR_PLUGINPATH_NAME={'set' if preset_plugin_path else 'unset'}"
            )
            print(f"  {label} -> {result}")
            return result
    raise AssertionError(f"arm produced no RESULT line:\n{completed.stdout}{completed.stderr}")


def main() -> int:
    print("ovphysx version:", ovphysx.__version__)
    print("Codeless schema packages:")
    for path in ovphysx.codeless_schema_paths():
        print("  ", path)

    print("\n--- RegisterPlugins() before any USD access: supported ---")
    clean = _arm("none", register=True, preset_plugin_path=False)
    # Subset check, matching tests/python_tests/test_codeless_schemas.py: ovphysx
    # ships at least physxSchema and omniUsdPhysicsDeformableSchema, and the set
    # may grow.
    assert clean["registered"] >= 2, clean
    assert clean["applied"], f"documented recipe must work in a clean process: {clean}"
    assert clean["fallback"] == SOLVER_POSITION_ITERATION_COUNT_FALLBACK, clean

    print("\n--- RegisterPlugins() after the registry is built: silently ineffective ---")
    # These pin OpenUSD's registry lifecycle, not an ovphysx defect: a late
    # RegisterPlugins() cannot repair an already-populated schema registry, and
    # it reports success while doing so. If an arm here ever starts applying,
    # OpenUSD changed its lifecycle -- revisit the ordering warning in
    # docs/physics_schemas.md before relaxing the assert.
    for pre_touch in ("registry", "stage"):
        late = _arm(pre_touch, register=True, preset_plugin_path=False)
        assert late["registered"] >= 2, late
        # The two signals a caller would naturally check, both misleading here.
        assert late["type_found"], late
        assert not late["applied"], (
            f"late RegisterPlugins() unexpectedly worked after pre_touch={pre_touch}; "
            f"OpenUSD's registry lifecycle may have changed: {late}"
        )

    print("\n--- Control: neither RegisterPlugins() nor PXR_PLUGINPATH_NAME ---")
    control = _arm("stage", register=False, preset_plugin_path=False)
    assert not control["applied"], f"schemas must not resolve without registration: {control}"

    print("\n--- PXR_PLUGINPATH_NAME preset before process start: the supported route ---")
    for pre_touch in ("registry", "stage"):
        preset = _arm(pre_touch, register=False, preset_plugin_path=True)
        assert preset["applied"], (
            f"PXR_PLUGINPATH_NAME must make the codeless schemas available even after "
            f"pre_touch={pre_touch}, with no RegisterPlugins() call: {preset}"
        )
        # The prim never authored this attribute, so a correct value can only
        # come from a parsed generatedSchema.usda.
        assert preset["fallback"] == SOLVER_POSITION_ITERATION_COUNT_FALLBACK, (
            f"expected the {FALLBACK_ATTRIBUTE} schema fallback "
            f"{SOLVER_POSITION_ITERATION_COUNT_FALLBACK}: {preset}"
        )

    print("\nCodeless schema registration ordering behaves as documented.")
    return 0


if __name__ == "__main__":
    try:
        import pxr  # noqa: F401
    except ImportError:
        # Stock usd-core has no wheel for some platforms (e.g. linux-aarch64),
        # matching register_codeless_schemas.py.
        print("usd-core is not available on this platform; skipping ordering regression.")
        sys.exit(0)

    if "--arm" in sys.argv:
        sys.exit(run_arm(sys.argv[sys.argv.index("--arm") + 1], "--register" in sys.argv))
    sys.exit(main())
