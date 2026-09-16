#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Generate the deformable OutputRead benchmark assets.

The envs are authored rather than cloned because ``ovphysx_clone`` cannot replicate a
deformable: ``IPhysxReplicator``'s object switch falls through ``default: break`` for
``PxDeformableVolume`` / ``PxDeformableSurface``, so cloning a subtree holding one
succeeds and silently produces N envs with no deformable in them.

Each ``env_<i>`` is an internal reference to a ``class`` template prim, so the file is one
env's worth of mesh data plus N lines, and USD retargets the template's relationship
targets into the referencing subtree, which gives every env its own material, as a
per-material read lane needs. The template being a ``class`` keeps it abstract, so it is
not parsed and simulated as an N+1th env.

TWO materials per env: the volume body binds the generic one, the surface body binds
``SurfaceMaterial``, which adds ``OmniPhysicsSurfaceDeformableMaterialAPI`` and
``PhysxSurfaceDeformableMaterialAPI``. Without ``surfaceThickness`` /
``surfaceBendStiffness`` a surface deformable simulates as a default-stiffness membrane
rather than as cloth, and the three surface-only material properties the read serves are
omitted on a volume material, so a generic-only asset cannot reach that path.

For volume bodies the row count is NOT what this script authors: the read serves the
PhysX SIMULATION mesh, which comes out of cooking, so the authored resolution steers it
but does not set it. Surface bodies are direct. Read the benchmark's ``rows=`` line for
what a size actually produced.

Regenerate:

    python gen_deformables.py

Requires no USD install to run. Validate the output with one if you have it:

    python -c "from pxr import Usd; Usd.Stage.Open('deformables_envs_128.usda')"
"""

import math
import os

# The size registered in OutputRead.cpp. Deformables are far more expensive per body than a
# rigid cube (to cook at load and to step), so this sits well below the rigid lane's counts.
# Compare it against the binding lane at the SAME size, not against readonly_rb.
ENV_COUNTS = (128,)

# Cells per axis of the volume body, and vertices per axis of the surface body. Chosen so
# one env carries a few dozen simulation nodes of each kind: enough that the gather has
# something to do, small enough that N bodies still cook and step in a benchmark pass.
VOLUME_CELLS = 2
SURFACE_VERTS = 5

ENV_SPACING = 4.0

# Kuhn decomposition of a cube into 6 tetrahedra: the six monotone paths from corner
# (0,0,0) to (1,1,1). Unlike the 5-tet decomposition it needs no per-cell alternation.
# Every cell uses the same six, and the shared faces still conform.
KUHN_PATHS = (
    ((0, 0, 0), (1, 0, 0), (1, 1, 0), (1, 1, 1)),
    ((0, 0, 0), (1, 0, 0), (1, 0, 1), (1, 1, 1)),
    ((0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 1, 1)),
    ((0, 0, 0), (0, 1, 0), (0, 1, 1), (1, 1, 1)),
    ((0, 0, 0), (0, 0, 1), (1, 0, 1), (1, 1, 1)),
    ((0, 0, 0), (0, 0, 1), (0, 1, 1), (1, 1, 1)),
)


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _signed_volume(points, tet):
    a, b, c, d = (points[i] for i in tet)
    return _dot(_cross(_sub(b, a), _sub(c, a)), _sub(d, a))


def build_volume(cells, size=1.0):
    """A `cells`^3 box of tetrahedra: points, tets, and the outward boundary triangles."""
    n = cells + 1
    step = size / cells

    def vid(i, j, k):
        return (i * n + j) * n + k

    points = []
    for i in range(n):
        for j in range(n):
            for k in range(n):
                # Centred on x/z and sitting above the origin on y, so an env's two bodies
                # share a column and the grid placement below is the only thing separating envs.
                points.append((i * step - size / 2.0, j * step + 1.0, k * step - size / 2.0))

    tets = []
    for i in range(cells):
        for j in range(cells):
            for k in range(cells):
                for path in KUHN_PATHS:
                    tet = tuple(vid(i + d[0], j + d[1], k + d[2]) for d in path)
                    # PhysX requires positive-volume tetrahedra. The Kuhn paths are consistent
                    # among themselves but their handedness depends on the axis order above,
                    # so fix it from the geometry rather than asserting it.
                    if _signed_volume(points, tet) < 0.0:
                        tet = (tet[1], tet[0], tet[2], tet[3])
                    tets.append(tet)

    # Boundary triangles, derived from the tets rather than from the box: a face shared by
    # two tets is interior, a face belonging to exactly one is on the surface. Deriving them
    # keeps the collision surface consistent with the decomposition above, which authoring
    # the six box faces independently would not guarantee.
    face_owner = {}
    for tet in tets:
        for tri in ((0, 1, 2), (0, 2, 3), (0, 3, 1), (1, 3, 2)):
            face = tuple(tet[t] for t in tri)
            key = tuple(sorted(face))
            if key in face_owner:
                face_owner[key] = None  # interior: seen twice
            else:
                face_owner[key] = (face, tet)

    surface = []
    for entry in face_owner.values():
        if entry is None:
            continue
        face, tet = entry
        # Wind outward: away from the tet's remaining vertex.
        opposite = next(v for v in tet if v not in face)
        a, b, c = (points[i] for i in face)
        if _dot(_cross(_sub(b, a), _sub(c, a)), _sub(points[opposite], a)) > 0.0:
            face = (face[0], face[2], face[1])
        surface.append(face)

    return points, tets, surface


def build_surface(verts, size=1.0):
    """A `verts` x `verts` cloth grid: points and triangles."""
    step = size / (verts - 1)
    points = []
    for i in range(verts):
        for j in range(verts):
            # Above the volume body, so the two never start interpenetrating.
            points.append((i * step - size / 2.0, 2.5, j * step - size / 2.0))

    tris = []
    for i in range(verts - 1):
        for j in range(verts - 1):
            v00 = i * verts + j
            v01 = v00 + 1
            v10 = v00 + verts
            v11 = v10 + 1
            tris.append((v00, v10, v01))
            tris.append((v01, v10, v11))
    return points, tris


def fmt_points(points):
    return "[" + ", ".join("(%g, %g, %g)" % p for p in points) + "]"


def fmt_zeros(count):
    return "[" + ", ".join(["(0, 0, 0)"] * count) + "]"


def fmt_int4(tets):
    return "[" + ", ".join("(%d, %d, %d, %d)" % t for t in tets) + "]"


def fmt_int3(tris):
    return "[" + ", ".join("(%d, %d, %d)" % t for t in tris) + "]"


def fmt_flat_int(tris):
    return "[" + ", ".join(str(i) for tri in tris for i in tri) + "]"


def generate(env_count):
    vpoints, tets, surface_tris = build_volume(VOLUME_CELLS)
    spoints, stris = build_surface(SURFACE_VERTS)

    side = int(math.ceil(math.sqrt(env_count)))
    envs = []
    for i in range(env_count):
        row, col = divmod(i, side)
        envs.append(
            '        def Xform "env_%d" (\n'
            "            prepend references = </_deformableEnv>\n"
            "        )\n"
            "        {\n"
            "            double3 xformOp:translate = (%g, 0, %g)\n"
            '            uniform token[] xformOpOrder = ["xformOp:translate"]\n'
            "        }\n" % (i, col * ENV_SPACING, row * ENV_SPACING)
        )

    return TEMPLATE % {
        "env_count": env_count,
        "volume_cells": VOLUME_CELLS,
        "volume_vertices": len(vpoints),
        "volume_tets": len(tets),
        "surface_vertices": len(spoints),
        "surface_tris": len(stris),
        "vpoints": fmt_points(vpoints),
        "vzeros": fmt_zeros(len(vpoints)),
        "tets": fmt_int4(tets),
        "surface_faces": fmt_int3(surface_tris),
        "spoints": fmt_points(spoints),
        "szeros": fmt_zeros(len(spoints)),
        "stris_int3": fmt_int3(stris),
        "stris_flat": fmt_flat_int(stris),
        "sface_counts": "[" + ", ".join(["3"] * len(stris)) + "]",
        "envs": "".join(envs),
    }


TEMPLATE = '''#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Y"
    doc = "GENERATED by gen_deformables.py -- do not edit by hand. %(env_count)d deformable envs, each one volume body (%(volume_cells)d^3 cells, %(volume_vertices)d authored vertices, %(volume_tets)d tets), one surface body (%(surface_vertices)d vertices, %(surface_tris)d triangles) and its own material. Envs are authored rather than cloned because the replicator does not copy deformables; see the script."
)

# The per-env template. A class, so it composes where it is referenced and the default
# traversal predicate skips it here -- it is not an N+1th env.
class Xform "_deformableEnv"
{
    def Material "Material" (
        prepend apiSchemas = ["PhysicsMaterialAPI", "OmniPhysicsDeformableMaterialAPI", "PhysxDeformableMaterialAPI"]
    )
    {
        float omniphysics:dynamicFriction = 0.5
        float omniphysics:poissonsRatio = 0.3
        float omniphysics:youngsModulus = 1000
        float physxDeformableMaterial:elasticityDamping = 0.01
    }

    def Material "SurfaceMaterial" (
        prepend apiSchemas = ["PhysicsMaterialAPI", "OmniPhysicsDeformableMaterialAPI", "OmniPhysicsSurfaceDeformableMaterialAPI", "PhysxDeformableMaterialAPI", "PhysxSurfaceDeformableMaterialAPI"]
    )
    {
        float omniphysics:dynamicFriction = 0.5
        float omniphysics:poissonsRatio = 0.3
        float omniphysics:youngsModulus = 1000
        float physxDeformableMaterial:elasticityDamping = 0.01
        float omniphysics:surfaceThickness = 1.0
        float omniphysics:surfaceBendStiffness = 10000
        float physxDeformableMaterial:bendDamping = 0.01
    }

    def TetMesh "Volume" (
        prepend apiSchemas = ["OmniPhysicsDeformableBodyAPI", "OmniPhysicsVolumeDeformableSimAPI", "OmniPhysicsDeformablePoseAPI:default", "PhysicsCollisionAPI", "MaterialBindingAPI"]
    )
    {
        bool omniphysics:deformableBodyEnabled = true
        point3f[] deformablePose:default:omniphysics:points = %(vpoints)s
        uniform token[] deformablePose:default:omniphysics:purposes = ["bindPose"]
        point3f[] omniphysics:restShapePoints = %(vpoints)s
        int4[] omniphysics:restTetVtxIndices = %(tets)s
        rel material:binding:physics = </_deformableEnv/Material> (
            bindMaterialAs = "weakerThanDescendants"
        )
        point3f[] points = %(vpoints)s
        int3[] surfaceFaceVertexIndices = %(surface_faces)s
        int4[] tetVertexIndices = %(tets)s
        vector3f[] velocities = %(vzeros)s
        quatf xformOp:orient = (1, 0, 0, 0)
        float3 xformOp:scale = (1, 1, 1)
        float3 xformOp:translate = (0, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
    }

    def Mesh "Surface" (
        prepend apiSchemas = ["OmniPhysicsDeformableBodyAPI", "OmniPhysicsSurfaceDeformableSimAPI", "OmniPhysicsDeformablePoseAPI:default", "PhysicsCollisionAPI", "MaterialBindingAPI"]
    )
    {
        bool omniphysics:deformableBodyEnabled = true
        point3f[] points = %(spoints)s
        int[] faceVertexIndices = %(stris_flat)s
        int[] faceVertexCounts = %(sface_counts)s
        token subdivisionScheme = "none"
        vector3f[] velocities = %(szeros)s
        point3f[] omniphysics:restShapePoints = %(spoints)s
        int3[] omniphysics:restTriVtxIndices = %(stris_int3)s
        point3f[] deformablePose:default:omniphysics:points = %(spoints)s
        uniform token[] deformablePose:default:omniphysics:purposes = ["bindPose"]
        rel material:binding:physics = </_deformableEnv/SurfaceMaterial> (
            bindMaterialAs = "weakerThanDescendants"
        )
        quatf xformOp:orient = (1, 0, 0, 0)
        float3 xformOp:scale = (1, 1, 1)
        float3 xformOp:translate = (0, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
    }
}

def Xform "World"
{
    # Same spelling as cartpole_probe.usda and cubes20_envs_gpu.usda: PhysxSceneAPI applied,
    # enableGPUDynamics + broadphaseType. There is no _cpu twin of this asset -- the read
    # refuses to serve a deformable without a CUDA context (buildDeformableGroups warns and
    # skips), so a CPU lane would report groups=0 rows=0 rather than a comparison.
    def PhysicsScene "physicsScene" (
        prepend apiSchemas = ["PhysxSceneAPI"]
    )
    {
        vector3f physics:gravityDirection = (0, -1, 0)
        float physics:gravityMagnitude = 9.81
        uint physxScene:timeStepsPerSecond = 60
        bool physxScene:enableGPUDynamics = true
        token physxScene:broadphaseType = "GPU"
    }

    # No ground plane, deliberately: the bodies fall freely for the length of a pass. That
    # keeps every step's state genuinely different (which is what preStep() is for) without
    # paying for contacts, which are not what these lanes measure.
    def Xform "envs"
    {
%(envs)s    }
}
'''


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    for env_count in ENV_COUNTS:
        path = os.path.join(here, "deformables_envs_%d.usda" % env_count)
        with open(path, "w", newline="\n") as handle:
            handle.write(generate(env_count))
        print("wrote %s (%.1f KB)" % (path, os.path.getsize(path) / 1024.0))


if __name__ == "__main__":
    main()
