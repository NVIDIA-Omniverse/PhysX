# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

# NOTE: This file is included verbatim in documentation via literalinclude.
# Tutorial marker comments below define the included range.

"""Register ovphysx's codeless PhysX USD schemas with a stock usd-core runtime.

This is the reference example for external authoring and validation tooling
(OMPE-86833). It uses stock ``usd-core`` from PyPI together with
``ovphysx``, registers the codeless PhysX USD schemas exposed by ovphysx, and
applies a PhysX API schema to a prim on an in-memory stage. It doubles as a CI
regression test for schema exposure: it asserts on every step and exits
non-zero on failure.

This runs in a fresh process, which is what makes ``RegisterPlugins()`` viable
here. In a process that already opened a stage or queried the schema registry,
registration is silently ineffective and ``PXR_PLUGINPATH_NAME`` must be preset
instead; ``schema_registry_ordering.py`` covers both orderings (NVBug 6530141).
"""

# [tutorial-start]
import sys

import ovphysx

try:
    from pxr import Plug, Tf, Usd
except ImportError:
    # Stock usd-core has no wheel for some platforms (e.g. linux-aarch64). The
    # codeless schemas still ship in the wheel; there is simply no stock USD
    # runtime to register them into here, so skip rather than fail.
    print("usd-core is not available on this platform; skipping codeless schema demo.")
    sys.exit(0)


def main() -> int:
    print("ovphysx version:", ovphysx.__version__)

    # Discover the codeless PhysX USD schema packages bundled with ovphysx.
    # Each path is a resources/ directory holding a codeless plugInfo.json
    # (Type=resource) and generatedSchema.usda -- no compiled library.
    schema_paths = ovphysx.codeless_schema_paths()
    print("Codeless schema packages:")
    for path in schema_paths:
        print("  ", path)

    # Register them with the stock usd-core runtime. This must happen before
    # anything in the process opens a stage or queries USD's schema registry --
    # that registry is built once, on first access, and a late call fails
    # silently. If USD may already be initialised (any DCC host), preset
    # PXR_PLUGINPATH_NAME before the process launches instead. Refer to
    # docs/physics_schemas.md.
    registry = Plug.Registry()
    registered = []
    for path in schema_paths:
        registered.extend(registry.RegisterPlugins(str(path)))
    registered_names = sorted(plugin.name for plugin in registered)
    print("Registered USD plugins:", registered_names)
    assert "physxSchema" in registered_names, registered_names
    assert "omniUsdPhysicsDeformableSchema" in registered_names, registered_names

    # Codeless schemas carry no compiled C++/Python bindings, so apply API
    # schemas by their schema identifier and query with the generic USD API.
    stage = Usd.Stage.CreateInMemory()
    prim = stage.DefinePrim("/World/Box", "Cube")
    assert prim.ApplyAPI("PhysxRigidBodyAPI"), "failed to apply PhysxRigidBodyAPI"
    assert prim.ApplyAPI("PhysxCollisionAPI"), "failed to apply PhysxCollisionAPI"

    rigid_body_type = Tf.Type.FindByName("PhysxSchemaPhysxRigidBodyAPI")
    assert rigid_body_type != Tf.Type.Unknown, "PhysxRigidBodyAPI not in schema registry"
    assert prim.HasAPI(rigid_body_type), "prim is missing PhysxRigidBodyAPI after apply"

    print("Applied API schemas:", list(prim.GetAppliedSchemas()))
    print("Codeless PhysX schema registration succeeded.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
# [tutorial-end]
