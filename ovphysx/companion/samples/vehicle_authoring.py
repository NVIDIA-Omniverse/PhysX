# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-VEHICLE-001
# @covers AC-1 AC-4
"""Vehicle property editor: stopped worker math, customer-USD apply and undo."""
import json
import os
from pathlib import Path
from ovphysx_companion._worker import run_worker, write_reply
import sys


def worker(request):
    import ctypes
    from ovphysx_companion.vehicle import VehicleMath
    service=VehicleMath(ctypes.CDLL(os.environ['OVPHYSX_LIB']))
    return {'masses':service.sprung_masses(request['points'],request['mass']),
            'frame':service.suspension_frame(request['body_pose'],request['wheel_pose'])}


def run():
    from ovphysx.utils import codeless
    from ovphysx_companion.usd import EditSession
    from ovphysx_companion.usd.vehicle import apply_authoring
    import ovphysx_companion
    codeless.register_schemas(verify=True)
    from pxr import Usd,UsdGeom,UsdPhysics
    stage=Usd.Stage.CreateInMemory();stage.DefinePrim('/World','Xform')
    car=stage.DefinePrim('/World/Car','Xform');codeless.apply_api(car,'PhysxVehicleAPI')
    car.SetCustomDataByKey('physxVehicle:referenceFrameIsCenterOfMass',False)
    wheel=stage.DefinePrim('/World/Car/Wheel','Xform');codeless.apply_api(wheel,'PhysxVehicleWheelAttachmentAPI')
    suspension=stage.DefinePrim('/World/Suspension','Xform');codeless.apply_api(suspension,'PhysxVehicleSuspensionAPI')
    UsdGeom.SetStageMetersPerUnit(stage,.01);UsdPhysics.SetStageKilogramsPerUnit(stage,1)
    with EditSession(stage) as session:
        source=session.source
        env=dict(os.environ);env['PYTHONPATH']=os.pathsep.join(filter(None,[str(Path(ovphysx_companion.__file__).parents[1]),env.get('PYTHONPATH')]))
        request={'points':[[-1,0,-2],[1,0,-2],[-1,0,2],[1,0,2]],'mass':1200,
                 'body_pose':[1,0,0,0,0,0,1],'wheel_pose':[2,0,3,0,0,0,1]}
        data = run_worker(Path(__file__).resolve(), request, env=env)
        patch=apply_authoring(session,'/World/Car',suspension_frames={'/World/Car/Wheel':data['frame']},
            sprung_masses={'/World/Suspension':data['masses'][0]},reference_frame='actor-local',source=source,layer=stage.GetRootLayer())
        assert tuple(wheel.GetAttribute('physxVehicleWheelAttachment:suspensionFramePosition').Get())==(100,0,300)
        patch.undo();patch.redo()
    return stage


if __name__=='__main__':
    if '--worker' in sys.argv: write_reply(worker(json.load(sys.stdin)))
    else: print(run().GetRootLayer().ExportToString())
