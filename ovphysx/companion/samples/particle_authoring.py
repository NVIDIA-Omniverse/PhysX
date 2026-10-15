# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-PARTICLES-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Particle command: capture -> isolated native sampler -> selected USD set -> undo.

The editor loads customer USD; the worker owns native runtime services. A host
can keep the worker alive and route its cancellation/drain protocol to Sampler.
"""
import json
import os
from pathlib import Path
from ovphysx_companion._worker import run_worker, write_reply
import sys
from dataclasses import asdict


def worker(request):
    from ovphysx import PhysX
    from ovphysx_companion.particles import SamplingInput,SamplingService
    PhysX.set_cpu_mode(True);physics=PhysX()
    try:
        with SamplingService(physics) as service,service.sampler(SamplingInput(tuple(request['source']),request['points'],request['triangles'])) as sampler:
            result=sampler.resample(radius=.3,volume_radius=.4,max_samples=1000).result(30)
            return asdict(result)
    finally:physics.destroy()


def run():
    from cooking import POINTS,TRIANGLES
    from ovphysx.utils import codeless
    from ovphysx_companion.usd import EditSession
    from ovphysx_companion.usd.particles import capture_sampling_input,apply_particle_set
    from ovphysx_companion.particles import Samples
    import ovphysx_companion
    codeless.register_schemas(verify=True)
    from pxr import Usd,UsdGeom
    stage=Usd.Stage.CreateInMemory();stage.DefinePrim('/World','Xform');UsdGeom.SetStageMetersPerUnit(stage,1)
    mesh=UsdGeom.Mesh.Define(stage,'/World/Emitter');mesh.CreatePointsAttr(POINTS)
    mesh.CreateFaceVertexCountsAttr([3]*len(TRIANGLES));mesh.CreateFaceVertexIndicesAttr([i for t in TRIANGLES for i in t])
    stage.DefinePrim('/World/System','PhysxParticleSystem')
    with EditSession(stage) as session:
        request=capture_sampling_input(session,'/World/Emitter')
        payload={'source':(request.source.session,request.source.revision),'points':request.points,'triangles':request.triangles}
        env=dict(os.environ);env['PYTHONPATH']=os.pathsep.join(filter(None,[str(Path(ovphysx_companion.__file__).parents[1]),env.get('PYTHONPATH')]))
        values = run_worker(Path(__file__).resolve(), payload, env=env)
        if tuple(values.pop('source'))!=payload['source']:raise RuntimeError('Worker replied to a different source request')
        values['points']=tuple(map(tuple,values['points']));values['triangles']=tuple(values['triangles']);values['weights']=tuple(map(tuple,values['weights']))
        result=Samples(source=request.source,**values)
        patch=apply_particle_set(session,result,path='/World/Particles',particle_system='/World/System',layer=stage.GetRootLayer(),owner='particle-command')
        patch.undo();assert not stage.GetPrimAtPath('/World/Particles')
        patch.redo()
    return stage


if __name__=='__main__':
    if '--worker' in sys.argv:write_reply(worker(json.load(sys.stdin)))
    else:print(run().GetRootLayer().ExportToString())
