#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Generate articulation_pileup.usda: a benchmark fixture with a row of
free-falling articulations of increasing link count, landing on an assorted
scatter of rigid cubes and spheres on a ground plane.

Adapted from omni.physxdemos.scenes.ArticulationDemo, but without the demo
context: no demo.Base, no helper room setup, no fixed joint anchoring the
articulation root. The first link of each chain is left as a floating root
so the whole chain falls under gravity.

Articulation layout:
  - 16 articulations arranged in a row along Y, spaced 2.75 m apart.
  - Articulation i has (3 + i) capsule links, i = 0..15 -> 3..18 links.
    The shortest chain has 3 links; each subsequent one adds a link.
  - Each link is a Capsule with axis = X (radius 0.10, height 0.30).
  - Links spaced 0.5 m apart along X, connected by revolute joints (axis Y).
  - Articulations start at z = 4 m, lying horizontally.
"""

import math
import random

# ---------------- Articulation configuration ----------------
NUM_ARTICULATIONS = 16
MIN_LINKS = 3  # articulation i has (MIN_LINKS + i) links; longest is 18

CAPSULE_RADIUS = 0.10
CAPSULE_HEIGHT = 0.30
LINK_SPACING = CAPSULE_HEIGHT + 2.0 * CAPSULE_RADIUS  # = 0.50

ARTICULATION_Y_SPACING = 2.75
ARTICULATION_Z = 4.0  # initial height

# ---------------- Rigid-body scatter configuration ----------------
SEED = 12345
NUM_CUBES = 50
NUM_SPHERES = 50
SCATTER_MIN_DIST = 1.0

# Ground plane sized with a wide buffer beyond the scatter area so obstacles
# pushed sideways by falling articulations are unlikely to roll off the edge.
_HALF_Y = (NUM_ARTICULATIONS - 1) * ARTICULATION_Y_SPACING / 2.0
GROUND_X = (-12.0, 12.0)
GROUND_Y = (-_HALF_Y - 5.0, _HALF_Y + 5.0)

# Inner area for scattering rigid bodies. Independent of ground so the
# obstacles cluster under the articulation drop zone regardless of how far
# the ground extends.
SCATTER_X = (-6.5, 6.5)
SCATTER_Y = (-_HALF_Y - 0.5, _HALF_Y + 0.5)


# ---------------- Templates ----------------
HEADER = """#usda 1.0
(
    defaultPrim = "World"
    endTimeCode = 1000
    metersPerUnit = 1
    startTimeCode = 0
    timeCodesPerSecond = 60
    upAxis = "Z"
)

def Xform "World"
{{
    def PhysicsScene "physicsScene" (
        prepend apiSchemas = ["PhysxSceneAPI"]
    )
    {{
        vector3f physics:gravityDirection = (0, 0, -1)
        float physics:gravityMagnitude = 9.81
        uint physxScene:timeStepsPerSecond = 240
    }}

    def Xform "GroundPlane"
    {{
        quatf xformOp:orient = (1, 0, 0, 0)
        float3 xformOp:scale = (1, 1, 1)
        double3 xformOp:translate = (0, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]

        def Plane "CollisionPlane" (
            prepend apiSchemas = ["PhysicsCollisionAPI"]
        )
        {{
            uniform token axis = "Z"
            uniform token purpose = "guide"
        }}

        def Mesh "GroundMesh"
        {{
            uniform bool doubleSided = 0
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            normal3f[] normals = [(0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1)]
            point3f[] points = [({gx0}, {gy0}, 0), ({gx1}, {gy0}, 0), ({gx1}, {gy1}, 0), ({gx0}, {gy1}, 0)]
            color3f[] primvars:displayColor = [(0.4, 0.4, 0.4)]
        }}
    }}
"""

CAPSULE_TEMPLATE = """
        def Capsule "{name}" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI"]
        )
        {{
            uniform token axis = "X"
            double height = {height}
            double radius = {radius}
            bool physics:collisionEnabled = 1
            bool physics:kinematicEnabled = 0
            bool physics:rigidBodyEnabled = 1
            color3f[] primvars:displayColor = [({r}, {g}, {b})]
            quatf xformOp:orient = (1, 0, 0, 0)
            float3 xformOp:scale = (1, 1, 1)
            double3 xformOp:translate = ({tx}, {ty}, {tz})
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
        }}
"""

REVOLUTE_JOINT_TEMPLATE = """
        def PhysicsRevoluteJoint "{name}"
        {{
            uniform token physics:axis = "Y"
            rel physics:body0 = <{body0}>
            rel physics:body1 = <{body1}>
            point3f physics:localPos0 = ({lp0}, 0, 0)
            point3f physics:localPos1 = ({lp1}, 0, 0)
            quatf physics:localRot0 = (1, 0, 0, 0)
            quatf physics:localRot1 = (1, 0, 0, 0)
            float physics:lowerLimit = -90
            float physics:upperLimit = 90
        }}
"""

ART_OPEN = """
    def Xform "{name}" (
        prepend apiSchemas = ["PhysicsArticulationRootAPI"]
    )
    {{
"""

ART_CLOSE = """    }
"""

CUBE_TEMPLATE = """
    def Cube "{name}" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI"]
    )
    {{
        double size = 1.0
        float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]
        bool physics:collisionEnabled = 1
        bool physics:kinematicEnabled = 0
        bool physics:rigidBodyEnabled = 1
        color3f[] primvars:displayColor = [({r}, {g}, {b})]
        quatd xformOp:orient = ({qw}, 0, 0, {qz})
        double3 xformOp:scale = ({sx}, {sy}, {sz})
        double3 xformOp:translate = ({tx}, {ty}, {tz})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
    }}
"""

SPHERE_TEMPLATE = """
    def Sphere "{name}" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI"]
    )
    {{
        double radius = {radius}
        bool physics:collisionEnabled = 1
        bool physics:kinematicEnabled = 0
        bool physics:rigidBodyEnabled = 1
        color3f[] primvars:displayColor = [({r}, {g}, {b})]
        quatd xformOp:orient = (1, 0, 0, 0)
        double3 xformOp:scale = (1, 1, 1)
        double3 xformOp:translate = ({tx}, {ty}, {tz})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
    }}
"""

FOOTER = """}

def Xform "Environment"
{
    def DistantLight "defaultLight"
    {
        float inputs:angle = 1
        float inputs:intensity = 3000
        quatd xformOp:orient = (0.6532814824381883, 0.2705980500730985, 0.27059805007309845, 0.6532814824381882)
        double3 xformOp:scale = (1, 1, 1)
        double3 xformOp:translate = (0, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
    }
}
"""


# ---------------- Emit helpers ----------------
def emit_articulation(idx, num_links, y_pos, z_pos):
    art_name = f"articulation_{idx:02d}"
    art_path = f"/World/{art_name}"
    out = [ART_OPEN.format(name=art_name)]

    total_len = num_links * LINK_SPACING
    x_start = -total_len / 2.0 + LINK_SPACING / 2.0

    for j in range(num_links):
        link_x = x_start + j * LINK_SPACING
        t = j / max(1, num_links - 1)
        r = 0.95 - 0.55 * t
        g = 0.20 + 0.70 * t
        b = 0.40
        out.append(
            CAPSULE_TEMPLATE.format(
                name=f"link_{j:02d}",
                height=CAPSULE_HEIGHT,
                radius=CAPSULE_RADIUS,
                tx=link_x,
                ty=y_pos,
                tz=z_pos,
                r=r,
                g=g,
                b=b,
            )
        )

    for j in range(1, num_links):
        out.append(
            REVOLUTE_JOINT_TEMPLATE.format(
                name=f"joint_{j:02d}",
                body0=f"{art_path}/link_{j-1:02d}",
                body1=f"{art_path}/link_{j:02d}",
                lp0=LINK_SPACING / 2.0,
                lp1=-LINK_SPACING / 2.0,
            )
        )

    out.append(ART_CLOSE)
    return "".join(out)


def main():
    out = [
        HEADER.format(
            gx0=GROUND_X[0],
            gx1=GROUND_X[1],
            gy0=GROUND_Y[0],
            gy1=GROUND_Y[1],
        )
    ]

    # Articulations of increasing link count.
    y_start = -(NUM_ARTICULATIONS - 1) * ARTICULATION_Y_SPACING / 2.0
    for i in range(NUM_ARTICULATIONS):
        num_links = MIN_LINKS + i
        y = y_start + i * ARTICULATION_Y_SPACING
        out.append(emit_articulation(i, num_links, y, ARTICULATION_Z))

    # Pre-decide each obstacle's size and visual props so we can use the
    # actual per-body bounding-circle radius when placing it: this is what
    # guarantees no plan-view overlap with any already-placed body, instead
    # of relying on a single global SCATTER_MIN_DIST.
    rng = random.Random(SEED)
    GAP = 0.05  # small extra clearance between bodies
    specs = []

    for i in range(NUM_CUBES):
        sx = rng.uniform(0.75, 1.65)
        sy = rng.uniform(0.75, 1.65)
        sz = rng.uniform(0.60, 1.50)
        yaw = rng.uniform(0.0, math.pi)
        # Bounding-circle radius of the rotated rectangular footprint.
        radius = 0.5 * math.sqrt(sx * sx + sy * sy)
        specs.append(
            (
                "cube",
                i,
                radius,
                dict(
                    sx=sx,
                    sy=sy,
                    sz=sz,
                    qw=math.cos(yaw / 2.0),
                    qz=math.sin(yaw / 2.0),
                    r=0.75 + 0.20 * rng.random(),
                    g=0.55 + 0.30 * rng.random(),
                    b=0.30 + 0.20 * rng.random(),
                ),
            )
        )
    for i in range(NUM_SPHERES):
        rad = rng.uniform(0.30, 0.90)
        specs.append(
            (
                "sphere",
                i,
                rad,
                dict(
                    radius=rad, r=0.30 + 0.30 * rng.random(), g=0.55 + 0.30 * rng.random(), b=0.80 + 0.20 * rng.random()
                ),
            )
        )

    # Place the largest bodies first; harder constraints win less random retries.
    specs.sort(key=lambda s: -s[2])

    placed = []  # (x, y, radius)
    skipped = []
    for kind, idx, radius, params in specs:
        for _ in range(800):
            x = rng.uniform(*SCATTER_X)
            y = rng.uniform(*SCATTER_Y)
            ok = True
            for px, py, pr in placed:
                min_d = radius + pr + GAP
                if (x - px) ** 2 + (y - py) ** 2 < min_d * min_d:
                    ok = False
                    break
            if ok:
                placed.append((x, y, radius))
                if kind == "cube":
                    out.append(
                        CUBE_TEMPLATE.format(
                            name=f"cube_{idx:02d}",
                            tx=x,
                            ty=y,
                            tz=params["sz"] / 2.0,
                            sx=params["sx"],
                            sy=params["sy"],
                            sz=params["sz"],
                            qw=params["qw"],
                            qz=params["qz"],
                            r=params["r"],
                            g=params["g"],
                            b=params["b"],
                        )
                    )
                else:
                    out.append(
                        SPHERE_TEMPLATE.format(
                            name=f"sphere_{idx:02d}",
                            radius=params["radius"],
                            tx=x,
                            ty=y,
                            tz=params["radius"],
                            r=params["r"],
                            g=params["g"],
                            b=params["b"],
                        )
                    )
                break
        else:
            skipped.append((kind, idx))

    if skipped:
        import sys

        sys.stderr.write(
            f"warning: skipped {len(skipped)} bodies that could not be "
            f"placed without overlap; consider a larger scatter area\n"
        )

    out.append(FOOTER)
    print("".join(out))


if __name__ == "__main__":
    main()
