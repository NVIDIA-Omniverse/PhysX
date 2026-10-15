# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-ATTACH-USD-001
# @covers AC-1 AC-2 AC-3 AC-4

"""Attachment extension flow: select, extract, apply computed pairs, undo/redo.

Copied points go to a native worker; customer USD stays in this editor process.
The native matcher computes pairs from geometry, then Python authors the schema.
"""

import os
from pathlib import Path
from ovphysx_companion._worker import run_worker

from ovphysx_companion.usd import EditSession, capture_mesh, apply_vertex_attachment, match_vertex_attachment


def native_matcher(source, target, distance):
    worker = Path(__file__).with_name("point_matching.py")
    import ovphysx_companion
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join(filter(None, [str(Path(ovphysx_companion.__file__).parents[1]),
                                                    env.get("PYTHONPATH")]))
    return run_worker(worker, dict(source=source, target=target, distance=distance), env=env)



def run():
    # Reuse the core schema-registration helper before constructing any stage.
    from ovphysx.utils import codeless
    codeless.register_schemas(verify=True)
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/World", "Xform")
    for path in ("/World/ClothA", "/World/ClothB"):
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr([(0, 0, 0), (1, 0, 0), (0, 1, 0)])
        mesh.CreateFaceVertexCountsAttr([3])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
        codeless.apply_api(mesh.GetPrim(), "OmniPhysicsDeformableBodyAPI")
        codeless.apply_api(mesh.GetPrim(), "OmniPhysicsSurfaceDeformableSimAPI")

    with EditSession(stage) as session:
        a, b = (capture_mesh(session, p) for p in ("/World/ClothA", "/World/ClothB"))
        result = match_vertex_attachment(a, b, distance=0.0, matcher=native_matcher)
        command = apply_vertex_attachment(session, result, path="/World/Seam",
                                          layer=stage.GetRootLayer(), owner="attachment-tool")
        # A Kit command's undo/redo methods call the same patch methods.
        command.undo()
        assert not stage.GetPrimAtPath("/World/Seam")
        command.redo()
        seam = stage.GetPrimAtPath("/World/Seam")
        assert seam.GetRelationship("omniphysics:src0").GetTargets()[0] == a.path
    return stage


if __name__ == "__main__":
    stage = run()
    print(stage.GetRootLayer().ExportToString())
