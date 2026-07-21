#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Generate a synthetic warehouse-style USDA fixture for ovphysx benchmarks.

Layout: a (ROWS x COLS) grid of warehouse racks. Each rack has four static
corner posts plus LEVELS levels of shelving, each shelf holding
ITEMS_PER_SHELF dynamic pallet-sized rigid bodies. Plus one large static
ground plane.

Default sizing targets ~2.4k dynamic rigid bodies + ~400 static colliders,
about 1.4x heavier than the Kapla fixture (1728 bodies). Sizing
constrained by the remote repo's 1 MiB per-file limit on text files —
the ovphysx subtree intentionally stores .usda as text (not LFS) per
ovphysx/.gitattributes. Tune ROWS / COLS / LEVELS / ITEMS_PER_SHELF
below to scale.

Usage:
    python3 tests/benchmarks/data/gen_warehouse.py > \
        tests/benchmarks/data/warehouse.usda
"""

import sys

# Sizing — defaults give 10 * 10 * 3 * 8 = 2400 dynamic items + 400 static.
ROWS = 10
COLS = 10
LEVELS = 3
ITEMS_PER_SHELF = 8

# Geometry (meters)
RACK_SPACING = 8.0     # distance between adjacent racks in X and Z
RACK_HALF = 2.5        # half-width of a rack (posts at +/- RACK_HALF in X and Z)
                       # — was 1.5 but that left items overlapping by 0.114m
                       # at startup (item size 0.4 vs spacing 2.0/7=0.286);
                       # the initial penetration biased step-time results.
RACK_HEIGHT = 1.8      # vertical spacing between shelf levels
POST_SIZE = 0.2        # cross-section of each post
ITEM_SIZE = 0.4        # cube edge of each pallet item — with RACK_HALF=2.5
                       # the per-shelf spacing is 4.0/7 ≈ 0.571m, leaving
                       # ~0.171m gap between adjacent items (no initial
                       # penetration).
GROUND_HALF = (max(ROWS, COLS) * RACK_SPACING) / 2.0 + 10.0


HEADER = f"""#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Y"
)

def Xform "World"
{{
    def PhysicsScene "physicsScene" (
        prepend apiSchemas = ["PhysxSceneAPI"]
    )
    {{
        vector3f physics:gravityDirection = (0, -1, 0)
        float physics:gravityMagnitude = 9.81
        uint physxScene:timeStepsPerSecond = 240
    }}

    def Cube "GroundPlane" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {{
        double size = 1.0
        double3 xformOp:scale = ({GROUND_HALF * 2}, 0.02, {GROUND_HALF * 2})
        double3 xformOp:translate = (0, -0.01, 0)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }}
"""

POST_TEMPLATE = """
    def Cube "{name}" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {{
        double size = 1.0
        double3 xformOp:scale = ({sx}, {sy}, {sz})
        double3 xformOp:translate = ({tx}, {ty}, {tz})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }}
"""

ITEM_TEMPLATE = """
    def Cube "{name}" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI"]
    )
    {{
        double size = {size}
        float physics:mass = 1.0
        double3 xformOp:translate = ({tx}, {ty}, {tz})
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}
"""

FOOTER = "}\n"


def main() -> int:
    out = [HEADER]

    grid_origin_x = -(COLS - 1) * RACK_SPACING / 2.0
    grid_origin_z = -(ROWS - 1) * RACK_SPACING / 2.0
    post_height = RACK_HEIGHT * LEVELS

    for r in range(ROWS):
        for c in range(COLS):
            cx = grid_origin_x + c * RACK_SPACING
            cz = grid_origin_z + r * RACK_SPACING
            # Four static corner posts per rack
            for pi, (dx, dz) in enumerate([(-RACK_HALF, -RACK_HALF),
                                            (RACK_HALF, -RACK_HALF),
                                            (-RACK_HALF, RACK_HALF),
                                            (RACK_HALF, RACK_HALF)]):
                out.append(POST_TEMPLATE.format(
                    name=f"post_r{r:02d}_c{c:02d}_p{pi}",
                    sx=POST_SIZE, sy=post_height, sz=POST_SIZE,
                    tx=cx + dx, ty=post_height / 2.0, tz=cz + dz,
                ))

            # Dynamic items on each shelf
            for level in range(LEVELS):
                shelf_y = (level + 0.5) * RACK_HEIGHT
                for i in range(ITEMS_PER_SHELF):
                    ix = cx - RACK_HALF + 0.5 + (i / max(ITEMS_PER_SHELF - 1, 1)) * (2 * RACK_HALF - 1.0)
                    out.append(ITEM_TEMPLATE.format(
                        name=f"item_r{r:02d}_c{c:02d}_l{level}_i{i:02d}",
                        size=ITEM_SIZE,
                        tx=ix, ty=shelf_y, tz=cz,
                    ))

    out.append(FOOTER)

    total_dynamic = ROWS * COLS * LEVELS * ITEMS_PER_SHELF
    total_static = ROWS * COLS * 4 + 1  # posts + ground
    sys.stderr.write(
        f"warehouse: {total_dynamic} dynamic + {total_static} static\n"
    )
    sys.stdout.write("".join(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
