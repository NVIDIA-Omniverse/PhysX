# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-QUERY-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-NATIVE-QUERY-001
# @covers AC-1 AC-2 AC-3 AC-4
import ctypes as C
from dataclasses import replace
from pathlib import Path
import pytest
from ovphysx_companion.queries import SceneQueries,Geometry,Mode,_Api,_Query,_Hit,_Result
from ovphysx_companion.inspection import ObjectKind
from ovphysx_companion.native import ToolsSession,CompanionError,Status
from ovphysx_companion.preview import PreviewSession


def test_raycast_core_parity_modes_filters_and_capacity(attached):
    from ovphysx import SceneQueryMode
    with ToolsSession(attached) as tools,SceneQueries(tools) as query:
        all_hits=query.raycast((0,0,10),(0,0,-1),20,mode=Mode.ALL)
        core=attached.raycast((0,0,10),(0,0,-1),20,mode=SceneQueryMode.ALL)
        assert len(all_hits.hits)==all_hits.total==len(core)==2
        assert [h.distance for h in all_hits.hits]==pytest.approx(sorted(h['distance'] for h in core))
        assert all_hits.hits[0].body==query.resolve('/World/BoxA')
        assert all_hits.hits[0].shape==query.resolve('/World/BoxA',ObjectKind.SHAPE)
        assert all_hits.hits[0].position[2]==pytest.approx(query.read(all_hits.hits[0].body).pose[2]+.5)
        assert all_hits.hits[0].normal==pytest.approx((0,0,1))
        assert all_hits.step==query.read(all_hits.hits[0].body).step and all_hits.revision>0
        closest=query.raycast((0,0,10),(0,0,-1),20)
        assert closest.hits==(all_hits.hits[0],) and not closest.truncated
        any_hit=query.raycast((0,0,10),(0,0,-1),20,mode=Mode.ANY)
        assert any_hit.any and any_hit.total==1 and len(any_hit.hits)==1
        truncated=query.raycast((0,0,10),(0,0,-1),20,mode=Mode.ALL,capacity=1)
        assert truncated.total==2 and truncated.truncated and truncated.hits==closest.hits
        body=query.resolve('/World/BoxA');floor=query.resolve('/World/Floor')
        assert query.raycast((0,0,10),(0,0,-1),20,include=[floor]).hits[0].body==floor
        assert query.raycast((0,0,10),(0,0,-1),20,exclude=[body]).hits[0].body==floor
        assert not query.raycast((0,0,10),(0,0,-1),20,include=[body],exclude=[body]).any
        assert query.raycast((0,0,10),(0,0,-1),20,dynamic=False).hits[0].body==floor
        assert query.raycast((0,0,10),(0,0,-1),20,static=False).hits[0].body==body
        assert not query.raycast((100,100,10),(0,0,-1),20).any
        with pytest.raises(CompanionError) as error:query.raycast((100,100,10),(0,0,-1),20,exclude=[replace(body,generation=body.generation+1)])
        assert error.value.status==Status.STALE


def test_sweep_overlap_primitives_bound_geometry_and_core(attached):
    from ovphysx import SceneQueryGeometryType,SceneQueryMode
    with ToolsSession(attached) as tools,SceneQueries(tools) as query:
        body=query.resolve('/World/BoxA')
        z=query.read(body).pose[2]
        sphere=query.sweep(Geometry.sphere(.25),(0,0,10),(0,0,-1),20,mode=Mode.ALL)
        assert sphere.hits[0].body==body and sphere.hits[0].distance==pytest.approx(10-z-.75,abs=.002)
        core=attached.sweep(SceneQueryGeometryType.SPHERE,(0,0,-1),20,mode=SceneQueryMode.ALL,radius=.25,position=(0,0,10))
        assert [v.distance for v in sphere.hits]==pytest.approx(sorted(v['distance'] for v in core))
        for geometry in (Geometry.sphere(.25),Geometry.box((.25,.25,.25)),Geometry.capsule(.25,.25),
                         Geometry.bound(query.resolve('/World/BoxA',ObjectKind.SHAPE))):
            overlap=query.overlap(geometry,(0,0,z))
            assert overlap.total==1 and overlap.hits[0].body==body
            assert overlap.hits[0].position is None and overlap.hits[0].distance is None and overlap.hits[0].normal is None
            assert query.overlap(geometry,(0,0,z),mode=Mode.ANY).any
            sweep=query.sweep(geometry,(0,0,10),(0,0,-1),20)
            assert sweep.hits[0].body==body and sweep.hits[0].distance>0
        bounded=query.overlap(Geometry.box((10,10,10)),(0,0,0),capacity=1)
        assert bounded.total==3 and bounded.truncated and len(bounded.hits)==1
        with pytest.raises(CompanionError) as error:query.overlap(Geometry.sphere(1),(0,0,0),mode=Mode.CLOSEST)
        assert error.value.status==Status.UNAVAILABLE


def test_query_point_instances_and_backface_triangles(physics,tmp_path):
    import ovstage
    path=tmp_path/'query-instances.usda'
    path.write_text('''#usda 1.0
(metersPerUnit = 1
 upAxis = "Z")
def Xform "World" {
    def PhysicsScene "Scene" {
        float physics:gravityMagnitude = 0
    }
    def PointInstancer "Instances" {
        rel prototypes = </World/Instances/Box>
        int[] protoIndices = [0, 0]
        point3f[] positions = [(0, 0, 3), (2, 0, 3)]
        quath[] orientations = [(1, 0, 0, 0), (1, 0, 0, 0)]
        def Cube "Box" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]) {
            double size = 1
        }
    }
    def Mesh "Convex" (prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]) {
        point3f[] points = [(-1,-1,-1), (1,-1,-1), (1,1,-1), (-1,1,-1), (-1,-1,1), (1,-1,1), (1,1,1), (-1,1,1)]
        int[] faceVertexCounts = [4,4,4,4,4,4]
        int[] faceVertexIndices = [0,3,2,1,4,5,6,7,0,1,5,4,1,2,6,5,2,3,7,6,3,0,4,7]
        token physics:approximation = "convexHull"
        double3 xformOp:translate = (10,0,0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
    def Mesh "Triangle" (prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]) {
        point3f[] points = [(-2,-2,0), (2,-2,0), (0,2,0)]
        int[] faceVertexCounts = [3]
        int[] faceVertexIndices = [0,1,2]
        token physics:approximation = "none"
        uniform token subdivisionScheme = "none"
        uniform bool doubleSided = false
    }
}
''')
    stage=ovstage.Stage('query-instances')
    try:
        ovstage.population.open_usd(stage,str(path),ordinal=1,domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        with PreviewSession(physics,stage,read_ordinal=1) as preview,SceneQueries(preview) as query:
            preview.step(.01)
            a=query.raycast((0,0,5),(0,0,-1),4).hits[0]
            b=query.raycast((2,0,5),(0,0,-1),4).hits[0]
            assert (a.body.instance,b.body.instance)==(0,1) and a.body.object!=b.body.object
            assert a.body.key==b.body.key and a.shape==b.shape
            assert a.body==query.resolve('/World/Instances/Box',instance=0)
            assert b.body==query.resolve('/World/Instances/Box',instance=1)
            assert query.read(a.body).pose[:3]==pytest.approx((0,0,3))
            assert query.read(b.body).pose[:3]==pytest.approx((2,0,3))
            assert not query.raycast((0,0,5),(0,0,-1),4,exclude=[a.body]).any
            assert query.raycast((2,0,5),(0,0,-1),4,exclude=[a.body]).any
            assert not query.raycast((2,0,5),(0,0,-1),4,exclude=[a.shape]).any
            front=query.raycast((0,0,1),(0,0,-1),2)
            assert front.any and front.hits[0].distance==pytest.approx(1) and front.hits[0].face==0
            assert not query.raycast((0,0,-1),(0,0,1),2).any
            assert query.raycast((0,0,-1),(0,0,1),2,both_sides=True).any
            convex=query.resolve('/World/Convex',ObjectKind.SHAPE)
            convex_sweep=query.sweep(Geometry.bound(convex),(0,0,8),(0,0,-1),10,include=[a.body])
            assert convex_sweep.hits[0].distance==pytest.approx(3.5,abs=.02)
            assert query.overlap(Geometry.bound(convex),(0,0,3),include=[a.body]).any
            triangle=query.resolve('/World/Triangle',ObjectKind.SHAPE)
            with pytest.raises(CompanionError) as error:query.overlap(Geometry.bound(triangle),(0,0,0))
            assert error.value.status==Status.UNAVAILABLE
    finally:stage.destroy()


def test_query_invalid_inputs_and_abi_preserve_output(attached):
    acquire=attached._lib.ovphysx_companion_queries_acquire
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
    table=_Api();C.memset(C.byref(table),0xa5,C.sizeof(table));before_table=bytes(table)
    assert acquire(2,C.sizeof(table),C.byref(table))==Status.ABI_MISMATCH and bytes(table)==before_table
    assert acquire(1,C.sizeof(table)-1,C.byref(table))==Status.ABI_MISMATCH and bytes(table)==before_table
    with ToolsSession(attached) as tools,SceneQueries(tools) as query:
        api=query._query_api
        request=_Query(0,0,0,3,1);request.pose[6]=1;request.direction[2]=-1;request.distance=10
        hit=_Hit();result=_Result();C.memset(C.byref(hit),0xa5,C.sizeof(hit));C.memset(C.byref(result),0xa5,C.sizeof(result))
        before=(bytes(hit),bytes(result))
        assert api.execute(tools.identity.attachment,C.byref(request),C.sizeof(request)-1,C.byref(hit),C.sizeof(hit),1,C.byref(result),C.sizeof(result))==Status.INVALID_ARGUMENT
        assert (bytes(hit),bytes(result))==before
        request.direction[2]=float('nan')
        assert api.execute(tools.identity.attachment,C.byref(request),C.sizeof(request),C.byref(hit),C.sizeof(hit),1,C.byref(result),C.sizeof(result))==Status.INVALID_ARGUMENT
        assert (bytes(hit),bytes(result))==before
        for op in (lambda:query.raycast((0,0,10),(0,0,0),10),lambda:query.raycast((0,0,10),(0,0,-2),10),
                   lambda:query.raycast((0,0,10),(0,0,-1),0),lambda:query.overlap(Geometry.sphere(-1),(0,0,0))):
            with pytest.raises(CompanionError) as error:op()
            assert error.value.status==Status.INVALID_ARGUMENT


def test_query_pending_owner_isolation_and_retirement(attached):
    import ovstage
    from concurrent.futures import ThreadPoolExecutor
    stage=ovstage.Stage('query-peer')
    try:
        ovstage.population.open_usd(stage,str(Path(__file__).resolve().parents[1]/'samples/scenes/two_boxes.usda'),ordinal=1,
                                    domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        with ToolsSession(attached) as tools,SceneQueries(tools) as playback:
            with PreviewSession(attached,stage,read_ordinal=1) as preview,SceneQueries(preview) as query:
                reference=query.resolve('/World/BoxA')
                preview.simulate(.01)
                with pytest.raises(CompanionError) as error:query.raycast((0,0,10),(0,0,-1),20)
                assert error.value.status==Status.BUSY
                assert playback.raycast((0,0,10),(0,0,-1),20).any
                preview.fetch()
                copied=query.raycast((0,0,10),(0,0,-1),20)
                assert copied.hits[0].body==reference
                assert query.path(reference)==playback.path(playback.resolve('/World/BoxA'))
                with pytest.raises(CompanionError) as error:playback.raycast((0,0,10),(0,0,-1),20,include=[reference])
                assert error.value.status==Status.STALE
                with ThreadPoolExecutor(1) as pool:
                    with pytest.raises(RuntimeError,match='host thread'):pool.submit(query.raycast,(0,0,10),(0,0,-1),20).result()
            assert copied.hits[0].position is not None
            with PreviewSession(attached,stage,read_ordinal=1) as newer,SceneQueries(newer) as query:
                with pytest.raises(CompanionError) as error:query.overlap(Geometry.sphere(1),(0,0,2),include=[reference])
                assert error.value.status==Status.STALE
    finally:stage.destroy()
