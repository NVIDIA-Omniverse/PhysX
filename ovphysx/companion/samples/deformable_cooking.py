# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-DEFORM-COOK-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-DEFORM-USD-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Volume authoring tool: native tetrahedralization, customer-USD apply, undo/redo.

The explicit worker contains native cooking; the editor process loads only the
customer's USD. Replace the one-shot subprocess with a retained authoring worker
and host command adapter when integrating with an extension.
"""
import json
import os
from pathlib import Path
from ovphysx_companion._worker import run_worker, write_reply
import sys


def worker(request):
    from ovphysx import PhysX
    from ovphysx_companion.cooking import CookingService
    from ovphysx_companion.deformable_cooking import cook_volume
    PhysX.set_cpu_mode(True)
    physics = PhysX()
    try:
        with CookingService(physics) as service:
            body = cook_volume(service, request['points'], request['triangles'], voxel_resolution=4).require_complete()
            def mesh(value):
                return dict(points=value.points.tolist(), elements=value.indices.tolist(), version=value.cache_key)
            return dict(simulation=mesh(body.simulation), collision=mesh(body.collision),
                        elements=body.visual_mapping.elements, weights=body.visual_mapping.weights)
    finally:
        physics.destroy()


def run():
    from cooking import POINTS, TRIANGLES
    from ovphysx.utils import codeless
    from ovphysx_companion.usd import (EditSession, DeformableMesh, DeformableBody,
                                      Embedding, apply_deformable_body)
    codeless.register_schemas(verify=True)
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim('/World', 'Xform')
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    with EditSession(stage) as session:
        source = session.source  # Keep the editor revision while the worker runs.
        import ovphysx_companion
        env = dict(os.environ)
        env['PYTHONPATH'] = os.pathsep.join(filter(None, [str(Path(ovphysx_companion.__file__).parents[1]),
                                                         env.get('PYTHONPATH')]))
        data = run_worker(Path(__file__).resolve(), dict(points=POINTS, triangles=TRIANGLES), env=env)
        def role(name, value):
            return DeformableMesh(name.title(), name, 'tetrahedron', tuple(map(tuple, value['points'])),
                                  tuple(map(tuple, value['elements'])), value['version'])
        simulation, collision = role('simulation', data['simulation']), role('collision', data['collision'])
        embedding = Embedding(simulation.topology_version, tuple(data['elements']), tuple(map(tuple, data['weights'])))
        visual = DeformableMesh('Visual', 'visual', 'triangle', POINTS, TRIANGLES, 'source-cube-v1', embedding=embedding)
        body = DeformableBody(source, (simulation, collision, visual))
        patch = apply_deformable_body(session, body, path='/World/SoftCube',
                                      layer=stage.GetRootLayer(), owner='deformable-tool')
        assert stage.GetPrimAtPath('/World/SoftCube/Simulation').HasAPI('OmniPhysicsVolumeDeformableSimAPI')
        patch.undo()
        assert not stage.GetPrimAtPath('/World/SoftCube')
        patch.redo()
    return stage


if __name__ == '__main__':
    if '--worker' in sys.argv:
        write_reply(worker(json.load(sys.stdin)))
    else:
        print(run().GetRootLayer().ExportToString())
