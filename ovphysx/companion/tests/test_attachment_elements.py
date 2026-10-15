# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-ATTACH-USD-001
# @covers AC-1 AC-2 AC-3 AC-4
import pytest
from pxr import Sdf, UsdGeom, UsdPhysics
from ovphysx.utils import codeless
from ovphysx_companion.usd.meshes import capture_mesh, capture_tet_mesh
from ovphysx_companion.usd.attachments import (
    ElementAttachment, ElementCollisionFilter, VertexAttachment, apply_element_attachment,
    apply_collision_filter, apply_vertex_attachment, remove_owned_attachment)
from ovphysx_companion.usd.session import StaleResultError


def volume(stage):
    mesh = UsdGeom.TetMesh.Define(stage, '/World/Volume')
    mesh.CreatePointsAttr([(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)])
    mesh.CreateTetVertexIndicesAttr([(0, 1, 2, 3)])
    mesh.CreateSurfaceFaceVertexIndicesAttr([(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)])
    codeless.apply_api(mesh.GetPrim(), 'OmniPhysicsDeformableBodyAPI')
    codeless.apply_api(mesh.GetPrim(), 'OmniPhysicsVolumeDeformableSimAPI')
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    return mesh


def test_volume_vertex_attachment_and_managed_removal_preserve_children(scene):
    stage, session = scene
    volume(stage)
    a, b = capture_mesh(session, '/World/A'), capture_tet_mesh(session, '/World/Volume')
    apply_vertex_attachment(session, VertexAttachment(a, b, (0,), (0,)),
                            path='/World/Seam', layer=stage.GetRootLayer(), owner='tool')
    prim = stage.GetPrimAtPath('/World/Seam')
    prim.CreateAttribute('user:note', Sdf.ValueTypeNames.String).Set('keep')
    stage.DefinePrim('/World/Seam/Unrelated', 'Scope')
    before = stage.GetRootLayer().ExportToString()
    patch = remove_owned_attachment(session, path='/World/Seam', layer=stage.GetRootLayer(), owner='tool')
    assert not prim.GetAttribute('omniphysics:attachmentEnabled').Get()
    assert stage.GetPrimAtPath('/World/Seam/Unrelated')
    assert prim.GetAttribute('user:note').Get() == 'keep'
    patch.undo()
    assert stage.GetRootLayer().ExportToString() == before
    patch.redo()
    with pytest.raises(ValueError, match='owned'):
        remove_owned_attachment(session, path='/World/Seam', layer=stage.GetRootLayer(), owner='foreign')


@pytest.mark.parametrize('kind', ['triangle', 'tetrahedron'])
def test_element_attachment_coordinate_domains_and_stale_rejection(scene, kind):
    stage, session = scene
    target = '/World/B'
    if kind == 'tetrahedron':
        volume(stage)
        target = '/World/Volume'
    a = capture_mesh(session, '/World/A')
    b = capture_tet_mesh(session, target) if kind == 'tetrahedron' else capture_mesh(session, target)
    result = ElementAttachment(a, b, (0, 1), (0, 0), ((.2, .3, .1), (.1, .2, .3)), kind)
    before = stage.GetRootLayer().ExportToString()
    patch = apply_element_attachment(session, result, path='/World/Seam', layer=stage.GetRootLayer(), owner='tool')
    prim = stage.GetPrimAtPath('/World/Seam')
    prefix = 'tri' if kind == 'triangle' else 'tet'
    assert tuple(prim.GetAttribute(f'omniphysics:{prefix}IndicesSrc1').Get()) == (0, 0)
    assert len(prim.GetAttribute(f'omniphysics:{prefix}CoordsSrc1').Get()) == 2
    patch.undo()
    assert stage.GetRootLayer().ExportToString() == before
    with pytest.raises(StaleResultError):
        apply_element_attachment(session, result, path='/World/Seam', layer=stage.GetRootLayer(), owner='tool')


def test_invalid_element_coordinates_are_atomic(scene):
    stage, session = scene
    volume(stage)
    a, b = capture_mesh(session, '/World/A'), capture_tet_mesh(session, '/World/Volume')
    before = stage.GetRootLayer().ExportToString()
    for indices, coords in (((1,), ((.2, .3, .1),)), ((0,), ((.5, .5, .5),)), ((0,), ((float('nan'), 0, 0),))):
        with pytest.raises(ValueError):
            apply_element_attachment(session, ElementAttachment(a, b, (0,), indices, coords, 'tetrahedron'),
                                     path='/World/Seam', layer=stage.GetRootLayer(), owner='tool')
        assert stage.GetRootLayer().ExportToString() == before


def test_collision_filter_paired_groups_and_surface_domain(scene):
    stage, session = scene
    volume(stage)
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath('/World/A'))
    a, b = capture_mesh(session, '/World/A'), capture_tet_mesh(session, '/World/Volume')
    result = ElementCollisionFilter(a, b, ((0,), ()), ((0, 2), (1,)))
    before = stage.GetRootLayer().ExportToString()
    patch = apply_collision_filter(session, result, path='/World/Filter', layer=stage.GetRootLayer(), owner='tool')
    prim = stage.GetPrimAtPath('/World/Filter')
    assert tuple(prim.GetAttribute('omniphysics:groupElemCounts0').Get()) == (1, 0)
    assert tuple(prim.GetAttribute('omniphysics:groupElemCounts1').Get()) == (2, 1)
    assert tuple(prim.GetAttribute('omniphysics:groupElemIndices1').Get()) == (0, 2, 1)
    patch.undo()
    assert stage.GetRootLayer().ExportToString() == before


def test_tet_topology_changes_invalidate_extracted_attachment(scene):
    stage, session = scene
    tet = volume(stage)
    a, b = capture_mesh(session, '/World/A'), capture_tet_mesh(session, '/World/Volume')
    tet.GetTetVertexIndicesAttr().Set([(0, 2, 1, 3)])
    with pytest.raises(StaleResultError):
        apply_vertex_attachment(session, VertexAttachment(a, b, (0,), (0,)),
                                path='/World/Seam', layer=stage.GetRootLayer(), owner='tool')


def test_native_element_and_filter_proposals_author_idempotently_and_reject_stale(scene):
    from dataclasses import replace
    from attachment_proposals import native_proposal
    from ovphysx_companion.usd.attachments import attachment_input,apply_attachment_proposal,apply_filter_proposal
    stage,session = scene
    tet = volume(stage)
    stage.GetPrimAtPath('/World/A').GetAttribute('points').Set([(.25,.25,.25),(.3,.25,.25),(.25,.3,.25)])
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath('/World/A'))
    foreign = stage.DefinePrim('/World/UserAttachment','OmniPhysicsVtxVtxAttachment')
    a,b = capture_mesh(session,'/World/A'),capture_tet_mesh(session,'/World/Volume')
    ea,eb = attachment_input(a,domain='triangle'),attachment_input(b,domain='tetrahedron')
    proposal = native_proposal(ea,eb,kind='attachment')
    assert proposal.coordinates[0] == pytest.approx((.25,.25,.25))
    destination = dict(path='/World/Seam',layer=stage.GetRootLayer(),owner='tool')
    apply_attachment_proposal(session,proposal,a,b,**destination)
    first = stage.GetRootLayer().ExportToString()
    a,b = capture_mesh(session,'/World/A'),capture_tet_mesh(session,'/World/Volume')
    ea,eb = attachment_input(a,domain='triangle'),attachment_input(b,domain='tetrahedron')
    proposal = native_proposal(ea,eb,kind='attachment')
    apply_attachment_proposal(session,proposal,a,b,**destination)
    assert stage.GetRootLayer().ExportToString() == first
    prim = stage.GetPrimAtPath('/World/Seam')
    assert tuple(prim.GetAttribute('omniphysics:tetCoordsSrc1').Get()[0]) == pytest.approx((.25,.25,.25))
    assert prim.GetRelationship('omniphysics:src1').GetTargets() == [Sdf.Path(b.path)]
    a,b = capture_mesh(session,'/World/A'),capture_tet_mesh(session,'/World/Volume')
    ea = attachment_input(a,domain='triangle')
    filters = native_proposal(ea,attachment_input(b,domain='triangle'),kind='filter',distance_meters=.003)
    patch = apply_filter_proposal(session,filters,a,b,path='/World/Filter',layer=stage.GetRootLayer(),owner='tool')
    assert stage.GetPrimAtPath('/World/Filter').GetAttribute('omniphysics:groupElemCounts0').Get()
    patch.undo()
    assert stage.GetRootLayer().ExportToString() == first
    with pytest.raises(StaleResultError):
        apply_attachment_proposal(session,proposal,a,b,**destination)
    patch.redo()
    remove_owned_attachment(session,**destination)
    assert foreign and not prim.GetAttribute('omniphysics:attachmentEnabled').Get()
    before = stage.GetRootLayer().ExportToString()
    with pytest.raises(StaleResultError,match='does not match'):
        apply_attachment_proposal(session,replace(proposal,source1=replace(eb,topology_version='other')),a,b,**destination)
    assert stage.GetRootLayer().ExportToString() == before


def test_transform_attachment_affine_anchors_managed_removal_and_stale(scene):
    from pxr import Gf
    from ovphysx_companion.usd.attachments import (
        capture_transform,match_transform_attachment,apply_transform_attachment)
    stage,session = scene
    rigid = UsdGeom.Xform.Define(stage,'/World/Rigid')
    matrix = Gf.Matrix4d(2,0,0,0, .5,3,0,0, 0,0,4,0, 5,6,7,1)
    op = rigid.AddTransformOp(); op.Set(matrix)
    UsdPhysics.RigidBodyAPI.Apply(rigid.GetPrim())
    a,b = capture_mesh(session,'/World/A'),capture_transform(session,'/World/Rigid')
    result = match_transform_attachment(a,b,vertex_indices=(0,2))
    for anchor,i in zip(result.local_positions,result.vertex_indices):
        assert tuple(matrix.Transform(Gf.Vec3d(*anchor))) == pytest.approx(a.points[i],abs=1e-6)
    destination = dict(path='/World/Anchor',layer=stage.GetRootLayer(),owner='tool')
    before = stage.GetRootLayer().ExportToString()
    patch = apply_transform_attachment(session,result,**destination)
    prim = stage.GetPrimAtPath('/World/Anchor')
    assert tuple(prim.GetAttribute('omniphysics:vtxIndicesSrc0').Get()) == (0,2)
    assert prim.GetTypeName() == 'OmniPhysicsVtxXformAttachment'
    first = stage.GetRootLayer().ExportToString()
    a,b = capture_mesh(session,'/World/A'),capture_transform(session,'/World/Rigid')
    result = match_transform_attachment(a,b,vertex_indices=(0,2))
    apply_transform_attachment(session,result,**destination)
    assert stage.GetRootLayer().ExportToString() == first
    patch.undo(); assert stage.GetRootLayer().ExportToString() == before
    patch.redo()
    remove_owned_attachment(session,**destination)
    assert not stage.GetPrimAtPath('/World/Anchor').GetAttribute('omniphysics:attachmentEnabled').Get()
    rigid = UsdGeom.Xform(stage.GetPrimAtPath('/World/Rigid'))
    rigid.GetOrderedXformOps()[0].Set(Gf.Matrix4d(1))
    unchanged = stage.GetRootLayer().ExportToString()
    with pytest.raises(StaleResultError):
        apply_transform_attachment(session,result,**destination)
    assert stage.GetRootLayer().ExportToString() == unchanged


def test_attachment_proposal_extension_sample():
    from attachment_proposals import run
    stage = run()
    assert stage.GetPrimAtPath('/World/Seam').GetTypeName() == 'OmniPhysicsVtxTetAttachment'
    assert stage.GetPrimAtPath('/World/Filter').GetTypeName() == 'OmniPhysicsElementCollisionFilter'
