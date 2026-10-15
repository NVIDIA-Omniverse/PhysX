# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-DESCRIPTION-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Inspector property model from customer USD; no physics initialization/attach."""
def run():
    from ovphysx.utils import codeless
    codeless.register_schemas(verify=True)
    from pxr import Usd, UsdGeom, UsdPhysics
    from ovphysx_companion.usd import EditSession, StaleResultError
    from ovphysx_companion.usd.descriptions import capture_descriptions
    stage=Usd.Stage.CreateInMemory()
    stage.DefinePrim('/World','Xform')
    body=UsdGeom.Cube.Define(stage,'/World/Body')
    UsdPhysics.CollisionAPI.Apply(body.GetPrim())
    joint=UsdPhysics.RevoluteJoint.Define(stage,'/World/Joint')
    joint.CreateBody1Rel().SetTargets([body.GetPath()])
    with EditSession(stage) as session:
        model=capture_descriptions(session).query('shape','joint')
        rows=[dict(path=d.path,family=d.family,geometry=d.type_name,
                   meters_per_unit=d.meters_per_unit,body1=d.targets('physics:body1')) for d in model.descriptions]
        body.GetSizeAttr().Set(4)
        try:
            model.validate(session)
        except StaleResultError:
            pass # Host schedules a new capture before accepting an edit.
        else:
            raise AssertionError('Inspector accepted a stale model')
    return rows


if __name__=='__main__':
    import json
    print(json.dumps(run()))
