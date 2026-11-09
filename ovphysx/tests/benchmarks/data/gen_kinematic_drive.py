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

The --articulated mode replaces the single rigid rider with an ARTICULATION
(PhysicsArticulationRootAPI root + a serial chain of links joined by revolute
joints) fixed-jointed to the kinematic slab, so the slab carries real
joint/binding state. --joints N parametrizes the chain length so the A-B import
gap can be read against joint count. The articulation rides along; Path B still
drives only the slab's kinematicTarget.

Usage:
    python3 tests/benchmarks/data/gen_kinematic_drive.py > \
        tests/benchmarks/data/kinematic_drive_cpu.usda
    python3 tests/benchmarks/data/gen_kinematic_drive.py --count 32 > \
        tests/benchmarks/data/kinematic_drive_heavy_cpu.usda
    python3 tests/benchmarks/data/gen_kinematic_drive.py --articulated --joints 16 > \
        tests/benchmarks/data/kinematic_drive_articulated_j16_cpu.usda
"""

import argparse
import sys

SLAB_SX = 6.0   # slab half-footprint scale
SLAB_SY = 1.0
SLAB_SZ = 6.0
GRID_PITCH = 16.0  # spacing between slab/rider pairs so they never interact

# Articulation chain geometry. Links are small boxes laid end-to-end along +X,
# anchored above the slab top and joined by revolute joints about Y.
LINK_LEN = 0.4       # full length of a link box along X
LINK_CROSS = 0.2     # cross-section (Y, Z) of a link box
LINK_HALF = LINK_LEN / 2.0


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


# Articulation root carrying a serial revolute chain, fixed-jointed to the
# kinematic slab so the whole articulation rides with it. The number of revolute
# joints is `joints`; the chain has joints+1 links. Anchored above the slab top
# and laid out along +X. physxRigidBody sleep/stabilization thresholds are zeroed
# so the chain stays awake (live joint state the import must rediscover).
def _articulation(idx, slab_tx, slab_tz, joints):
    nlinks = joints + 1
    base_y = SLAB_SY / 2.0 + LINK_CROSS / 2.0 + 0.02  # just above the slab top
    root_path = f"/World/Articulation{idx}"
    parts = []
    parts.append(f"""
    def Xform "Articulation{idx}" (
        prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]
    )
    {{
        bool physxArticulation:enabledSelfCollisions = 0
        int physxArticulation:solverPositionIterationCount = 4
        int physxArticulation:solverVelocityIterationCount = 1
""")
    # Links laid end-to-end along +X starting at the slab origin.
    for li in range(nlinks):
        lx = slab_tx + li * LINK_LEN
        parts.append(f"""
        def Cube "link{li}" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]
        )
        {{
            double size = 1
            float physics:mass = 0.1
            float physxRigidBody:sleepThreshold = 0.0
            float physxRigidBody:stabilizationThreshold = 0.0
            double3 xformOp:translate = ({lx}, {base_y}, {slab_tz})
            double3 xformOp:scale = ({LINK_LEN}, {LINK_CROSS}, {LINK_CROSS})
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        }}
""")
    # Fixed joint anchoring link0 to the kinematic slab: the articulation base is
    # carried by the slab, so every import re-discovers this slab<->articulation
    # binding plus the internal joint chain.
    parts.append(f"""
        def PhysicsFixedJoint "rootJoint"
        {{
            rel physics:body0 = </World/KinematicSlab{idx}>
            rel physics:body1 = <{root_path}/link0>
            point3f physics:localPos0 = (0, {base_y}, 0)
            point3f physics:localPos1 = (0, 0, 0)
            quatf physics:localRot0 = (1, 0, 0, 0)
            quatf physics:localRot1 = (1, 0, 0, 0)
        }}
""")
    # Revolute joints about Y connecting consecutive links. localPos in the
    # scaled link frame: the box spans [-0.5, 0.5] in local size-1 units, so the
    # joint anchors at the link ends (+/-0.5).
    for ji in range(joints):
        parts.append(f"""
        def PhysicsRevoluteJoint "joint{ji}" (
            prepend apiSchemas = ["PhysicsDriveAPI:angular"]
        )
        {{
            uniform token physics:axis = "Y"
            rel physics:body0 = <{root_path}/link{ji}>
            rel physics:body1 = <{root_path}/link{ji + 1}>
            point3f physics:localPos0 = (0.5, 0, 0)
            point3f physics:localPos1 = (-0.5, 0, 0)
            quatf physics:localRot0 = (1, 0, 0, 0)
            quatf physics:localRot1 = (1, 0, 0, 0)
            float physics:lowerLimit = -45
            float physics:upperLimit = 45
            float drive:angular:physics:damping = 10
            float drive:angular:physics:stiffness = 50
            float drive:angular:physics:targetPosition = 0
        }}
""")
    parts.append("    }\n")
    return "".join(parts)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=1,
                        help="number of slab/rider pairs (default 1)")
    parser.add_argument("--articulated", action="store_true",
                        help="replace the rigid rider with an articulated (jointed) rider")
    parser.add_argument("--joints", type=int, default=16,
                        help="with --articulated: number of revolute joints in the rider chain (default 16)")
    args = parser.parse_args()
    if args.count < 1:
        sys.stderr.write("--count must be >= 1\n")
        return 2
    if args.articulated and args.joints < 1:
        sys.stderr.write("--joints must be >= 1\n")
        return 2

    if args.articulated:
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
            out.append(_articulation(i, tx, tz, args.joints))
        out.append("}\n")
        sys.stdout.write("".join(out))
        sys.stderr.write(f"kinematic_drive: {args.count} slab + articulated rider "
                         f"({args.joints} joints / {args.joints + 1} links each), CPU dynamics\n")
        return 0

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
