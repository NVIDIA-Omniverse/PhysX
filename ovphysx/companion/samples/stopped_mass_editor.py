# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-MASS-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Extension-style selection -> copied worker computation -> USD command/undo.

Set OVPHYSX_LIB to the existing build's core library. Customer USD remains in the
editor process. No simulation is attached or stepped by either process.
"""
import json
import os
import sys
from dataclasses import asdict
from pathlib import Path
import ovphysx_companion
from ovphysx_companion.mass import Result
from ovphysx_companion.native import MassProperties
from ovphysx_companion._worker import run_worker, write_reply


def compute_in_worker(request):
    data=asdict(request)
    data.pop('source')
    env=dict(os.environ)
    env['PYTHONPATH']=os.pathsep.join(filter(None,[str(Path(ovphysx_companion.__file__).parents[1]),env.get('PYTHONPATH')]))
    properties=run_worker(Path(__file__).resolve(),data,env=env)
    for name in ('center_of_mass','diagonal_inertia','principal_axes','bounds_min','bounds_max'):properties[name]=tuple(properties[name])
    return Result(MassProperties(**properties),request.source,request.body_path,tuple(s.path for s in request.shapes))


def run():
    from pxr import Usd,UsdGeom,UsdPhysics
    from ovphysx_companion.usd import EditSession
    from ovphysx_companion.usd.mass import capture_mass_request,apply_mass_result
    stage=Usd.Stage.CreateInMemory();UsdGeom.SetStageMetersPerUnit(stage,1);UsdPhysics.SetStageKilogramsPerUnit(stage,1)
    body=UsdGeom.Xform.Define(stage,'/Body');UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    cube=UsdGeom.Cube.Define(stage,'/Body/Collider');cube.CreateSizeAttr(2);cube.AddScaleOp().Set((1,2,3))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim());UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateDensityAttr(10)
    with EditSession(stage) as session:
        request=capture_mass_request(session,'/Body')
        result=compute_in_worker(request)
        patch=apply_mass_result(session,result,layer=stage.GetRootLayer())
        assert body.GetPrim().GetAttribute('physics:mass').Get()==480
        patch.undo();patch.redo() # host command stack owns this patch
    return stage

def worker():
    import ctypes
    from ovphysx_companion.mass import Shape, Overrides, Request, compute_mass_properties
    value=json.load(sys.stdin)
    value['shapes']=tuple(Shape(**(shape|{'overrides':Overrides(**shape['overrides'])})) for shape in value['shapes'])
    value['overrides']=Overrides(**value['overrides'])
    result=compute_mass_properties(ctypes.CDLL(os.environ['OVPHYSX_LIB']),Request(**value))
    write_reply(asdict(result.properties))


if __name__=='__main__':
    if '--worker' in sys.argv:worker()
    else:print(run().GetRootLayer().ExportToString())
