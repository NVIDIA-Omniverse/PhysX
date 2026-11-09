#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Generate the kinematic-drive microbenchmark fixture (AAuthoringKinematicDrive.cpp).

A minimal CPU scene (physxScene:enableGPUDynamics = false) with, per unit, one
KINEMATIC slab driven every tick and one RESTING DYNAMIC cube riding on top of
it. The rider carries material bindings and a resting contact against the slab,
so the OVStage control-plane path (ovphysx_update_from_ovstage) re-runs real
binding/contact discovery each import rather than looking artificially cheap.

The default (--count 1) emits one slab + one rider (the representative delta).
--count N emits N stacked slab/rider pairs on a grid so the import cost scales
with discovered state, for the "higher-count variant" called for when the
single-pair gap is small.

Usage:
    python3 tests/benchmarks/data/gen_kinematic_drive.py > \
        tests/benchmarks/data/kinematic_drive_cpu.usda
    python3 tests/benchmarks/data/gen_kinematic_drive.py --count 32 > \
        tests/benchmarks/data/kinematic_drive_heavy_cpu.usda
"""

import argparse
import sys

SLAB_SX = 6.0   # slab half-footprint scale
SLAB_SY = 1.0
SLAB_SZ = 6.0
GRID_PITCH = 16.0  # spacing between slab/rider pairs so they never interact


def _header() -> str:
    return """#usda 1.0
(
    defaultPrim = "World"
    doc = "Kinematic-drive microbenchmark fixture: per unit one kinematic slab plus one resting dynamic rider, CPU dynamics. See gen_kinematic_drive.py."
    metersPerUnit = 1
    upAxis = "Y"
    kilogramsPerUnit = 1
)

def Xform "World"
{
    def PhysicsScene "PhysicsScene" (
        prepend apiSchemas = ["PhysxSceneAPI"]
    )
    {
        vector3f physics:gravityDirection = (0, -1, 0)
        float physics:gravityMagnitude = 9.81
        bool physxScene:enableGPUDynamics = false
        string physxScene:broadphaseType = "MBP"
    }

    def Material "HighFriction" (
        prepend apiSchemas = ["PhysicsMaterialAPI"]
    )
    {
        float physics:staticFriction = 1.0
        float physics:dynamicFriction = 1.0
        float physics:restitution = 0.0
    }
"""


_SLAB_TEMPLATE = """
    def Cube "KinematicSlab{idx}" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsRigidBodyAPI", "PhysicsMassAPI", "MaterialBindingAPI"]
    )
    {{
        rel material:binding:physics = </World/HighFriction> (
            bindMaterialAs = "weakerThanDescendants"
        )
        bool physics:kinematicEnabled = true
        float physics:mass = 10.0
        double size = 1
        double3 xformOp:scale = ({sx}, {sy}, {sz})
        double3 xformOp:translate = ({tx}, 0, {tz})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }}
"""


# Resting dynamic rider sitting on top of the slab. Sleep/stabilization disabled
# so it stays awake and in the active set, and its resting contact against the
# slab is live state the import path must rediscover.
_RIDER_TEMPLATE = """
    def Cube "DynamicRider{idx}" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI", "MaterialBindingAPI"]
    )
    {{
        rel material:binding:physics = </World/HighFriction> (
            bindMaterialAs = "weakerThanDescendants"
        )
        float physics:mass = 1.0
        float physxRigidBody:sleepThreshold = 0.0
        float physxRigidBody:stabilizationThreshold = 0.0
        double size = 1
        double3 xformOp:translate = ({tx}, {ry}, {tz})
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=1,
                        help="number of slab/rider pairs (default 1)")
    args = parser.parse_args()
    if args.count < 1:
        sys.stderr.write("--count must be >= 1\n")
        return 2

    # Rider rests on the slab top: slab centered at y=0 with half-height SLAB_SY/2,
    # rider (unit cube, half-height 0.5) sits just above so it settles into contact.
    rider_y = SLAB_SY / 2.0 + 0.5

    side = 1
    while side * side < args.count:
        side += 1

    out = [_header()]
    for i in range(args.count):
        gx = i % side
        gz = i // side
        tx = gx * GRID_PITCH
        tz = gz * GRID_PITCH
        out.append(_SLAB_TEMPLATE.format(idx=i, sx=SLAB_SX, sy=SLAB_SY, sz=SLAB_SZ, tx=tx, tz=tz))
        out.append(_RIDER_TEMPLATE.format(idx=i, tx=tx, ry=rider_y, tz=tz))
    out.append("}\n")
    sys.stdout.write("".join(out))
    sys.stderr.write(f"kinematic_drive: {args.count} slab/rider pair(s), CPU dynamics\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
