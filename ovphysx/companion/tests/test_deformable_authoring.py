# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-DEFORM-USD-001
# @covers AC-1 AC-2 AC-3 AC-4
from dataclasses import replace
import pytest
from pxr import Sdf, UsdGeom, UsdPhysics, UsdShade
from ovphysx_companion.usd import (DeformableMesh, DeformableBody, Embedding, apply_deformable_body,
                                  capture_tet_mesh, MeshFrame, write_simulation_output)
from ovphysx_companion.usd.session import StaleResultError


def test_native_cook_worker_to_customer_usd_construction():
    from deformable_cooking import run
    stage = run()
    simulation = UsdGeom.TetMesh(stage.GetPrimAtPath('/World/SoftCube/Simulation'))
    collision = UsdGeom.TetMesh(stage.GetPrimAtPath('/World/SoftCube/Collision'))
    visual = stage.GetPrimAtPath('/World/SoftCube/Visual')
    assert len(simulation.GetTetVertexIndicesAttr().Get()) > 0
    assert len(collision.GetTetVertexIndicesAttr().Get()) > 0
    assert len(visual.GetAttribute('ovphysxCompanion:embedding:elements').Get()) == 8
    assert len(simulation.GetPointsAttr().Get()) != len(collision.GetPointsAttr().Get())


def recipe(session, *, surface=False):
    points = ((0, 0, 0), (1, 0, 0), (0, 1, 0)) if surface else ((0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1))
    elements = ((0, 1, 2),) if surface else ((0, 1, 2, 3),)
    sim = DeformableMesh('Simulation', 'simulation', 'triangle' if surface else 'tetrahedron',
                          points, elements, 'topology-v1', collision=True)
    visual_points = ((.2, .2, 0), (.6, .2, 0), (.2, .6, 0))
    weights = ((.6, .2, .2), (.2, .6, .2), (.2, .2, .6))
    if not surface:
        weights = tuple(w + (0.,) for w in weights)
    visual = DeformableMesh('Visual', 'visual', 'triangle', visual_points, ((0, 1, 2),),
                           'visual-v1', embedding=Embedding('topology-v1', (0, 0, 0), weights))
    return DeformableBody(session.source, (sim, visual))


@pytest.mark.parametrize('surface', [False, True])
def test_deformable_role_rest_bind_material_and_undo(scene, surface):
    stage, session = scene
    body = recipe(session, surface=surface)
    before = stage.GetRootLayer().ExportToString()
    patch = apply_deformable_body(session, body, path='/World/Body', layer=stage.GetRootLayer(), owner='cook')
    sim = stage.GetPrimAtPath('/World/Body/Simulation')
    visual = stage.GetPrimAtPath('/World/Body/Visual')
    assert sim.HasAPI('OmniPhysicsSurfaceDeformableSimAPI' if surface else 'OmniPhysicsVolumeDeformableSimAPI')
    assert sim.HasAPI(UsdPhysics.CollisionAPI)
    assert len(sim.GetAttribute('omniphysics:restShapePoints').Get()) == len(body.meshes[0].points)
    assert len(visual.GetAttribute('deformablePose:default:omniphysics:points').Get()) == 3
    assert tuple(visual.GetAttribute('ovphysxCompanion:embedding:elements').Get()) == (0, 0, 0)
    assert UsdShade.MaterialBindingAPI(sim).ComputeBoundMaterial(materialPurpose='physics')[0]
    after = stage.GetRootLayer().ExportToString()
    patch.undo()
    assert stage.GetRootLayer().ExportToString() == before
    patch.redo()
    assert stage.GetRootLayer().ExportToString() == after


def test_reconstruct_preserves_unowned_data_and_invalidates_embeddings(scene):
    stage, session = scene
    body = recipe(session)
    apply_deformable_body(session, body, path='/World/Body', layer=stage.GetRootLayer(), owner='cook')
    visual = stage.GetPrimAtPath('/World/Body/Visual')
    visual.CreateAttribute('user:comment', Sdf.ValueTypeNames.String).Set('keep')
    source = session.source
    old_snapshot = capture_tet_mesh(session, '/World/Body/Simulation')
    before = stage.GetRootLayer().ExportToString()
    invalid = replace(body, source=source, meshes=(replace(body.meshes[0], elements=((1,0,2,3),)), body.meshes[1]))
    with pytest.raises(ValueError, match='new topology version'):
        apply_deformable_body(session, invalid, path='/World/Body', layer=stage.GetRootLayer(), owner='cook')
    assert stage.GetRootLayer().ExportToString() == before
    updated = replace(body, source=source, meshes=(replace(body.meshes[0], topology_version='v2'),
                                                 replace(body.meshes[1], embedding=None)))
    apply_deformable_body(session, updated, path='/World/Body', layer=stage.GetRootLayer(), owner='cook')
    assert visual.GetAttribute('user:comment').Get() == 'keep'
    assert visual.GetAttribute('ovphysxCompanion:embedding:elements').Get() is None
    with pytest.raises(StaleResultError):
        session.validate(old_snapshot.source)


def test_bad_deformable_data_is_atomic_and_foreign_prims_are_preserved(scene):
    stage, session = scene
    body = recipe(session)
    before = stage.GetRootLayer().ExportToString()
    invalid = replace(body, meshes=(body.meshes[0], replace(body.meshes[1], embedding=Embedding('wrong', (0,), ((1,0,0,0),)))))
    with pytest.raises(ValueError, match='different simulation'):
        apply_deformable_body(session, invalid, path='/World/Body', layer=stage.GetRootLayer(), owner='cook')
    assert stage.GetRootLayer().ExportToString() == before
    stage.DefinePrim('/World/Body', 'Xform')
    with pytest.raises(ValueError, match='helper-owned'):
        apply_deformable_body(session, replace(body, source=session.source), path='/World/Body', layer=stage.GetRootLayer(), owner='cook')


def test_tet_output_publication_preserves_rest_topology(scene):
    import numpy as np
    stage, session = scene
    apply_deformable_body(session, recipe(session), path='/World/Body', layer=stage.GetRootLayer(), owner='cook')
    snapshot = capture_tet_mesh(session, '/World/Body/Simulation')
    prim = stage.GetPrimAtPath(snapshot.path)
    rest = tuple(map(tuple, prim.GetAttribute('omniphysics:restShapePoints').Get()))
    patch = write_simulation_output(session, [MeshFrame(snapshot, np.array(snapshot.points) + .1, 1, .01)],
                                    layer=stage.GetRootLayer(), time_code=1)
    assert tuple(map(tuple, prim.GetAttribute('omniphysics:restShapePoints').Get())) == rest
    assert tuple(map(tuple, prim.GetAttribute('tetVertexIndices').Get())) == snapshot.tetrahedra
    patch.undo()


def test_native_embedding_and_mapped_visual_output_to_customer_usd(scene):
    import numpy as np
    from ovphysx import _bindings
    from ovphysx_companion.mesh import embed_points
    from ovphysx_companion.native import Identity
    from ovphysx_companion.output import DeformableOutput
    from ovphysx_companion.usd.meshes import capture_mesh
    from ovphysx_companion.usd.deformables import embedding_from_mapping
    from ovphysx_companion.usd.writeback import map_deformable_output
    stage, session = scene
    data = recipe(session)
    simulation, visual = data.meshes
    mapping = embed_points(_bindings._lib, simulation.points, simulation.elements, visual.points,
                           element_width=4, topology_version=simulation.topology_version,
                           frame='simulation-local', units='stage')
    data = replace(data, meshes=(simulation, replace(visual, embedding=embedding_from_mapping(mapping))))
    apply_deformable_body(session, data, path='/World/Body', layer=stage.GetRootLayer(), owner='native-cook')
    UsdGeom.Xformable(stage.GetPrimAtPath('/World/Body/Visual')).AddTranslateOp().Set((.5, 0, 0))
    sim = capture_tet_mesh(session, '/World/Body/Simulation')
    dst = capture_mesh(session, '/World/Body/Visual')
    owner = Identity(7, 13)
    output = DeformableOutput(owner, sim.path, np.asarray(sim.points)+[0,0,1],
                              np.tile([0,0,2], (4,1)).astype(float), 1, .01)
    frame = map_deformable_output(session, output, sim, dst, mapping, library=_bindings._lib,
                                  topology_version=simulation.topology_version, expected_owner=owner)
    np.testing.assert_allclose(frame.positions, np.asarray(visual.points)+[-.5,0,1], atol=1e-7)
    np.testing.assert_allclose(frame.velocities, np.tile([0,0,2], (3,1)))
    bad_matrix = np.eye(4)
    bad_matrix[0,3] = .1
    with pytest.raises(ValueError, match='affine'):
        map_deformable_output(session, output, sim, replace(dst, local_to_world=tuple(map(tuple, bad_matrix))), mapping,
                              library=_bindings._lib, topology_version=simulation.topology_version, expected_owner=owner)
    before = stage.GetRootLayer().ExportToString()
    patch = write_simulation_output(session, [frame], layer=stage.GetRootLayer(), time_code=1)
    assert len(stage.GetPrimAtPath(dst.path).GetAttribute('points').Get(1)) == 3
    patch.undo()
    assert stage.GetRootLayer().ExportToString() == before
    sim = capture_tet_mesh(session, '/World/Body/Simulation')
    dst = capture_mesh(session, '/World/Body/Visual')
    with pytest.raises(ValueError, match='foreign runtime'):
        map_deformable_output(session, output, sim, dst, mapping, library=_bindings._lib,
                              topology_version=simulation.topology_version, expected_owner=Identity(7,14))


def test_surface_offset_velocities_use_native_normal_derivative_and_frame_conversion(scene):
    import numpy as np
    from ovphysx import _bindings
    from ovphysx_companion.mesh import embed_points
    from ovphysx_companion.native import Identity
    from ovphysx_companion.output import DeformableOutput
    from ovphysx_companion.usd.meshes import capture_mesh
    from ovphysx_companion.usd.writeback import map_deformable_output
    stage, session = scene
    body = recipe(session, surface=True)
    sim_data, visual = body.meshes
    queries = tuple((x,y,.5) for x,y,_ in visual.points)
    visual = replace(visual, points=queries, embedding=None)
    apply_deformable_body(session, replace(body, meshes=(sim_data,visual)), path='/World/Body',
                          layer=stage.GetRootLayer(), owner='surface')
    UsdGeom.Xformable(stage.GetPrimAtPath('/World/Body/Simulation')).AddScaleOp().Set((2,3,4))
    sim = capture_mesh(session, '/World/Body/Simulation')
    dst = capture_mesh(session, '/World/Body/Visual')
    mapping = embed_points(_bindings._lib, sim.points, sim_data.elements, queries, element_width=3,
                           topology_version='surface', max_distance=.5, frame='simulation-local', units='stage')
    owner = Identity(5,8)
    data = DeformableOutput(owner, sim.path, np.asarray(sim.points), np.asarray(((0,0,0),(0,0,8),(0,0,0)),float), 1,.01)
    frame = map_deformable_output(session,data,sim,dst,mapping,library=_bindings._lib,
                                  topology_version='surface',expected_owner=owner)
    np.testing.assert_allclose(frame.positions, np.asarray(queries)*[2,3,4], atol=2e-7)
    np.testing.assert_allclose(frame.velocities, [(-2,0,8*x) for x,y,z in queries], atol=3e-7)


@pytest.mark.parametrize('surface',[False,True])
def test_separate_simulation_and_collision_both_bind_physics_material(scene,surface):
    stage,session=scene
    body=recipe(session,surface=surface)
    sim=replace(body.meshes[0],collision=False)
    collision=replace(sim,name='Collision',role='collision')
    body=replace(body,meshes=(sim,collision,body.meshes[1]))
    apply_deformable_body(session,body,path='/World/Body',layer=stage.GetRootLayer(),owner='cook')
    bindings=[]
    for name in ('Simulation','Collision'):
        prim=stage.GetPrimAtPath('/World/Body/'+name)
        material=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')[0]
        assert material;bindings.append(material.GetPath())
    assert bindings[0]==bindings[1]
    assert not stage.GetPrimAtPath('/World/Body/Simulation').HasAPI(UsdPhysics.CollisionAPI)
