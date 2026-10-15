# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-ATTACH-COMPUTE-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-ATTACH-USD-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Extension-style element/filter authoring with a separate native worker.

The editor owns customer USD and undo. The worker owns copied geometry and
computes proposals without an attachment. A production host can replace this
one-shot transport with a persistent job service; no USD/native pointers cross it.
"""
from dataclasses import asdict
import json
import os
from pathlib import Path
from ovphysx_companion._worker import run_worker, write_reply
import sys


def worker(request):
    from ovphysx import _bindings
    from ovphysx_companion.attachments import MeshEndpoint,propose_attachment,propose_filter
    endpoints = []
    for value in request['endpoints']:
        values = dict(value,source=tuple(value['source']))
        endpoints.append(MeshEndpoint(**values))
    options = request['options']
    operation = propose_attachment if request['kind'] == 'attachment' else propose_filter
    result = asdict(operation(_bindings._lib,*endpoints,**options))
    # Echo the exact input identity/domain. The editor checks it before accepting
    # an asynchronous response; it separately validates current USD revisions.
    return dict(endpoints=request['endpoints'],result=result)


def native_proposal(source0,source1,*,kind,**options):
    import ovphysx_companion
    from ovphysx_companion.attachments import AttachmentProposal,FilterProposal
    from ovphysx_companion.mesh import ElementMap
    values = []
    for endpoint in (source0,source1):
        value = asdict(endpoint)
        value['source'] = [endpoint.source.session,endpoint.source.revision]
        values.append(value)
    request = dict(endpoints=values,kind=kind,options=options)
    env = dict(os.environ)
    env['PYTHONPATH'] = os.pathsep.join(filter(None,[str(Path(ovphysx_companion.__file__).parents[1]),env.get('PYTHONPATH')]))
    response = run_worker(Path(__file__).resolve(), request, env=env)
    if response['endpoints'] != json.loads(json.dumps(values)):
        raise ValueError('Worker returned a different source identity/topology')
    data = response['result']
    if kind == 'filter':
        return FilterProposal(source0,source1,tuple(map(tuple,data['pairs'])),data['distance_meters'])
    mapping = data['mapping']
    for name in ('topology','weights'):
        mapping[name] = tuple(map(tuple,mapping[name]))
    for name in ('elements','distances','normal_offsets'):
        mapping[name] = tuple(mapping[name])
    return AttachmentProposal(source0,source1,data['kind'],tuple(data['vertex_indices']),tuple(data['element_indices']),
                              tuple(map(tuple,data['coordinates'])),ElementMap(**mapping),
                              tuple(data['unmatched_vertices']),tuple(data['projected_vertices']))


def run():
    from ovphysx.utils import codeless
    codeless.register_schemas(verify=True)
    from pxr import Usd,UsdGeom,UsdPhysics
    from ovphysx_companion.usd import EditSession,capture_mesh
    from ovphysx_companion.usd.meshes import capture_tet_mesh
    from ovphysx_companion.usd.attachments import attachment_input,apply_attachment_proposal,apply_filter_proposal
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim('/World','Xform')
    cloth = UsdGeom.Mesh.Define(stage,'/World/Cloth')
    cloth.CreatePointsAttr([(.25,.25,.25),(.3,.25,.25),(.25,.3,.25)])
    cloth.CreateFaceVertexCountsAttr([3]); cloth.CreateFaceVertexIndicesAttr([0,1,2])
    volume = UsdGeom.TetMesh.Define(stage,'/World/Volume')
    volume.CreatePointsAttr([(0,0,0),(1,0,0),(0,1,0),(0,0,1)])
    volume.CreateTetVertexIndicesAttr([(0,1,2,3)])
    volume.CreateSurfaceFaceVertexIndicesAttr([(0,2,1),(0,1,3),(0,3,2),(1,2,3)])
    for mesh,api in ((cloth,'OmniPhysicsSurfaceDeformableSimAPI'),(volume,'OmniPhysicsVolumeDeformableSimAPI')):
        codeless.apply_api(mesh.GetPrim(),'OmniPhysicsDeformableBodyAPI')
        codeless.apply_api(mesh.GetPrim(),api)
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    with EditSession(stage) as session:
        a,b = capture_mesh(session,'/World/Cloth'),capture_tet_mesh(session,'/World/Volume')
        ea,eb = attachment_input(a,domain='triangle'),attachment_input(b,domain='tetrahedron')
        proposal = native_proposal(ea,eb,kind='attachment')
        apply_attachment_proposal(session,proposal,a,b,path='/World/Seam',layer=stage.GetRootLayer(),owner='attachment-tool')
        # Authored edits advance the source revision. Capture again before a
        # second command, even when the endpoint geometry itself is unchanged.
        a,b = capture_mesh(session,'/World/Cloth'),capture_tet_mesh(session,'/World/Volume')
        ea = attachment_input(a,domain='triangle')
        surface = attachment_input(b,domain='triangle')
        filters = native_proposal(ea,surface,kind='filter',distance_meters=.003)
        patch = apply_filter_proposal(session,filters,a,b,path='/World/Filter',layer=stage.GetRootLayer(),owner='attachment-tool')
        patch.undo(); patch.redo()
    return stage


if __name__ == '__main__':
    if '--worker' in sys.argv:
        write_reply(worker(json.load(sys.stdin)))
    else:
        print(run().GetRootLayer().ExportToString())
