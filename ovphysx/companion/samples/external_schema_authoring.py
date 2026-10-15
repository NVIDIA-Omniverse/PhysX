# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-NATIVE-SCHEMA-001
# @covers AC-1 AC-2
"""Customer-USD schemas and explicit native-population parameter columns.

This verifies data ingestion; native solver/contact registration is separate.
"""
from pathlib import Path
import json
import sys
import tempfile
from ovphysx_companion._worker import run_worker, write_reply
from ovphysx_companion.schemas import (ExternalSchema, SchemaBundle, register_customer_schemas,
                                       register_population_schemas, population_description)


def sample_bundle():
    return SchemaBundle(Path(__file__).parent/'native/schema/resources/plugInfo.json', (
        ExternalSchema('CompanionSpringJoint','joint',('companion:stiffness','companion:damping','companion:restLength')),
        ExternalSchema('CompanionBallAPI','geometry',('companion:radius',))))


def populate(path):
    import ovstage
    bundle = sample_bundle()
    register_population_schemas([bundle])
    result = {}
    with ovstage.Stage('external-schema-parameters') as stage:
        ovstage.population.open_usd_with_desc(stage, str(path), ordinal=1, time=0,
                                               descs=[population_description([bundle])])
        stage.advance_write_floor(ordinal=1).wait()
        with ovstage.PathDictionary(stage) as dictionary:
            for path, name in (('/World/Spring','companion:stiffness'), ('/World/Spring','companion:damping'),
                               ('/World/Ball','companion:radius')):
                with dictionary.create_path_list_from_strings([path]) as paths:
                    with stage.query_from_path_list(paths) as query:
                        with stage.read_attributes(query, [dictionary.intern_token(name)], ovstage.OrdinalRange(1,1)) as read:
                            values = []
                            for group in read.groups():
                                with group:
                                    values.extend(group.array(0).reshape(-1).tolist())
                            if len(values) != 1:
                                raise RuntimeError(f'Population lost external parameter {path}.{name}: {values}')
                            result[path+'.'+name] = values[0]
    return result


def main():
    register_customer_schemas([sample_bundle()])
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage,'/World')
    UsdPhysics.Scene.Define(stage,'/World/Scene')
    ball = UsdGeom.Sphere.Define(stage,'/World/Ball').GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(ball)
    UsdPhysics.CollisionAPI.Apply(ball)
    assert ball.ApplyAPI('CompanionBallAPI')
    assert ball.GetAttribute('companion:radius').Set(.75)
    joint = stage.DefinePrim('/World/Spring','CompanionSpringJoint')
    assert joint.IsA(UsdPhysics.Joint)
    assert joint.GetRelationship('physics:body1').SetTargets(['/World/Ball'])
    assert joint.GetAttribute('companion:stiffness').Set(125.)
    # Damping is intentionally unauthored: population must retain its schema default.
    with tempfile.TemporaryDirectory(prefix='companion-external-schema-') as temp:
        scene = Path(temp)/'scene.usda'
        stage.GetRootLayer().Export(str(scene))
        result = run_worker(Path(__file__).resolve(), str(scene))
    expected = {'/World/Spring.companion:stiffness':125., '/World/Spring.companion:damping':2.,
                '/World/Ball.companion:radius':.75}
    assert result == expected, result
    return result


if __name__ == '__main__':
    if '--worker' in sys.argv:write_reply(populate(json.load(sys.stdin)))
    else:print(json.dumps(main(), sort_keys=True))
