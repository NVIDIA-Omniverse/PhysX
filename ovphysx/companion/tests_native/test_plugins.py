# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CUSTOM-LIFETIME-001
# @covers AC-2 AC-3 AC-5 AC-6
# @implements REQ-COMP-NATIVE-ABI-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-JOINT-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-GEOMETRY-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-NATIVE-SCHEMA-001
# @covers AC-2 AC-3 AC-4
# @implements REQ-COMP-OVSTAGE-001
# @covers AC-2
import ctypes as C
from pathlib import Path
import subprocess
import sys
import pytest
from ovphysx_companion.plugins import NativePlugin, _Api
from ovphysx_companion.native import CompanionError, Status, ToolsSession
from ovphysx_companion.preview import PreviewSession
from ovphysx_companion.schemas import ExternalSchema, SchemaBundle, population_description

COMPANION = Path(__file__).resolve().parents[1]
BUNDLE = SchemaBundle(COMPANION/'samples/native/schema/resources/plugInfo.json', (
    ExternalSchema('CompanionSpringJoint','joint',('companion:stiffness','companion:damping','companion:restLength')),
    ExternalSchema('CompanionBallAPI','geometry',('companion:radius',))))


@pytest.fixture(scope='module')
def plugins(tmp_path_factory):
    build = tmp_path_factory.mktemp('compiled-companion-plugins')
    sdk = COMPANION.parent/'_install'
    contract=(sdk/'include/ovphysx_companion/native_build.h').read_text()
    configuration='Debug' if '#define OVPHYSX_COMPANION_DEBUG_BUILD 1' in contract else 'Release'
    subprocess.run(['cmake','-S',str(COMPANION/'samples/native'),'-B',str(build),
                    f'-DOVPHYSX_SDK={sdk}',f'-DCMAKE_BUILD_TYPE={configuration}','-DCOMPANION_BUILD_NEGATIVE_FIXTURES=ON'],
                   check=True,capture_output=True,text=True)
    subprocess.run(['cmake','--build',str(build),'--config',configuration,'--parallel','5'],
                   check=True,capture_output=True,text=True)
    def module(name):
        suffix = '.dll' if sys.platform == 'win32' else '.so'
        matches = list(build.rglob(('*' if sys.platform != 'win32' else '')+name+suffix))
        assert len(matches)==1, matches
        return matches[0]
    return module


def counts(path):
    library=C.CDLL(str(path))
    library.companion_sample_count.argtypes=[C.c_uint32]
    library.companion_sample_count.restype=C.c_uint32
    return library,lambda i:library.companion_sample_count(i)


def stage(name):
    import ovstage
    result=ovstage.Stage(name)
    try:
        ovstage.population.open_usd_with_desc(result,str(COMPANION/'samples/scenes/native_extensions.usda'),
            ordinal=1,time=0,descs=[population_description([BUNDLE])])
        result.advance_write_floor(ordinal=1).wait()
        return result
    except BaseException:
        result.destroy()
        raise


def test_plugin_table_rejects_size_version_without_writing(physics):
    table=_Api();C.memset(C.byref(table),0xa5,C.sizeof(table));before=bytes(table)
    acquire=physics._lib.ovphysx_companion_native_acquire
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
    assert acquire(2,C.sizeof(table),C.byref(table))==Status.ABI_MISMATCH
    assert bytes(table)==before
    assert acquire(1,C.sizeof(table)-1,C.byref(table))==Status.ABI_MISMATCH
    assert bytes(table)==before


@pytest.mark.parametrize('kind,status',[('VERSION',Status.ABI_MISMATCH),('BUILD',Status.ABI_MISMATCH),
                                        ('COMPILER',Status.ABI_MISMATCH),('ALIGNMENT',Status.INVALID_ARGUMENT),
                                        ('CALLBACK',Status.INVALID_ARGUMENT),('OUTPUT_PARAM',Status.INVALID_ARGUMENT)])
def test_incompatible_plugin_never_starts(physics,plugins,kind,status):
    path=plugins('companion_bad_'+kind)
    library,count=counts(path)
    assert count(3)==0
    with pytest.raises(CompanionError) as error:
        NativePlugin(physics,path)
    assert error.value.status==status
    assert count(3)==0 and count(4)==0


def test_plugin_lifetime_without_objects(physics,plugins):
    import ovphysx
    bootstrap=ovphysx.PhysX()
    module=NativePlugin(bootstrap,plugins('companion_spring'))
    try:
        bootstrap.destroy()
        assert physics._lib.ovphysx_shutdown().status != 0
        with pytest.raises(CompanionError):
            NativePlugin(physics,plugins('companion_spring'))
    finally:
        module.close();bootstrap.destroy()
    module.close()
    with NativePlugin(physics,plugins('companion_spring')):
        pass


def test_solver_contacts_queries_and_busy_retirement(physics,plugins):
    import ovstage
    spring_library,spring=counts(plugins('companion_spring'))
    ball_library,ball=counts(plugins('companion_ball'))
    before_s=[spring(i) for i in range(5)]
    before_b=[ball(i) for i in range(5)]
    with NativePlugin(physics,plugins('companion_spring')) as spring_module, NativePlugin(physics,plugins('companion_ball')) as ball_module:
        a=stage('native-core');b=stage('native-preview')
        preview=None
        try:
            physics.wait_task(physics.attach_ovstage(a,read_ordinal=1))
            physics.step(1/120);physics.wait_all()
            preview=PreviewSession(physics,b,read_ordinal=1,gpu_pipeline=0)
            preview.step(1/120)
            assert spring(0)==before_s[0]+2
            assert ball(0)==before_b[0]+2
            assert spring(2)>before_s[2]
            identity=preview.identity
            assert not spring_module.joint_always_update(identity,'/World/Spring')
            spring_module.set_joint_always_update(identity,'/World/Spring',True)
            assert spring_module.joint_always_update(identity,'/World/Spring')
            spring_module.set_joint_always_update(identity,'/World/Spring',False)
            assert not spring_module.joint_always_update(identity,'/World/Spring')
            spring_module.mark_joint_dirty(identity,'/World/Spring')
            refreshes=spring(5)
            spring_module.refresh_joint(identity,'/World/Spring')
            assert spring(5)==refreshes+1
            preview.simulate(1/120)
            with pytest.raises(CompanionError) as error:
                spring_module.refresh_joint(identity,'/World/Spring')
            assert error.value.status==Status.BUSY
            assert spring(5)==refreshes+1
            preview.fetch()
            import numpy as np
            def edit_stiffness(value, ordinal):
                with ovstage.PathDictionary(b) as dictionary:
                    with dictionary.create_path_list_from_strings(['/World/Spring']) as paths:
                        with b.query_from_path_list(paths) as query:
                            b.write_attribute(query, 'companion:stiffness', ordinal,
                                np.array([value], dtype=np.float32), is_array=False).wait()
                b.advance_write_floor(ordinal=ordinal).wait()
                preview.update(ordinal, ordinal)
            spring_library.companion_sample_last_refresh.restype=C.c_float
            edit_stiffness(250,2)
            spring_module.refresh_joint(identity,'/World/Spring')
            assert spring_library.companion_sample_last_refresh()==250
            with ToolsSession(physics) as tools:
                spring_module.refresh_joint(tools.identity,'/World/Spring')
                assert spring_library.companion_sample_last_refresh()==125
            edit_stiffness(-1,3)
            before_rejection=spring(5)
            with pytest.raises(CompanionError) as error:
                spring_module.refresh_joint(identity,'/World/Spring')
            assert error.value.status==Status.ERROR
            assert spring(5)==before_rejection
            edit_stiffness(250,4)
            spring_module.refresh_joint(identity,'/World/Spring')
            start=preview.rigid_pose('/World/Body').position
            # Authored native radius is .75, while the ordinary USD Sphere is .25.
            hits=physics.raycast(origin=[5,5,0],direction=[0,-1,0],distance=10)
            assert len(hits)==1
            assert 2.2 < hits[0]['distance'] < 2.3
            for module in (spring_module,ball_module):
                with pytest.raises(CompanionError) as error:
                    module.close()
                assert error.value.status==Status.BUSY
            for _ in range(180):
                preview.step(1/120)
            after=preview.rigid_pose('/World/Body').position
            resting=preview.rigid_pose('/World/Ball').position
            assert after[0] < start[0] - .5 # actual solver rows, not registration alone
            assert .70 < resting[1] < .85 # custom radius and generated plane contacts
            assert ball(2)>before_b[2]
            physics.wait_task(physics.detach_ovstage())
            assert spring(1)==before_s[1]+1 and ball(1)==before_b[1]+1
            with pytest.raises(CompanionError) as error:
                spring_module.close()
            assert error.value.status==Status.BUSY
            preview.close();preview=None
            assert spring(1)==before_s[1]+2 and ball(1)==before_b[1]+2
        finally:
            if preview:preview.close()
            if physics._attached_ovstage is not None: physics.wait_task(physics.detach_ovstage())
            a.destroy();b.destroy()
    assert spring(4)==before_s[4]+1 and ball(4)==before_b[4]+1


def test_flow_output_never_becomes_custom_joint_input(physics,plugins):
    # Custom-physics inputs share the companion: prefix. Skipping the output
    # ordinal does not hide a stored value from a later reparse, so output must
    # live in a subnamespace no input declares.
    import numpy as np
    import ovstage
    from ovphysx_companion.exchange import OutputColumn, StageFlow
    spring_library,_=counts(plugins('companion_spring'))
    spring_library.companion_sample_last_refresh.restype=C.c_float
    with NativePlugin(physics,plugins('companion_spring')) as spring_module:
        a=stage('native-flow-output')
        try:
            physics.wait_task(physics.attach_ovstage(a,read_ordinal=1))
            physics.step(1/120);physics.wait_all()
            with ToolsSession(physics) as tools, StageFlow(tools,a,consumed_ordinal=1) as flow:
                with pytest.raises(ValueError,match='companion:output:'):
                    OutputColumn('companion:stiffness',np.array([250],dtype=np.float32),'joint','N/m')
                flow.publish(2,['/World/Spring'],
                             [OutputColumn('companion:output:stiffness',np.array([250],dtype=np.float32),'joint','N/m')])
                # An unrelated authored body edit makes the core reparse inputs.
                with ovstage.PathDictionary(a) as dictionary:
                    with dictionary.create_path_list_from_strings(['/World/Body']) as paths:
                        with a.query_from_path_list(paths) as query:
                            a.write_attribute(query,'physics:density',3,np.array([10],dtype=np.float32),is_array=False).wait()
                a.advance_write_floor(ordinal=3).wait()
                assert flow.consume(3)
                spring_module.refresh_joint(tools.identity,'/World/Spring')
                assert spring_library.companion_sample_last_refresh()==125
        finally:
            if physics._attached_ovstage is not None: physics.wait_task(physics.detach_ovstage())
            a.destroy()


def test_extension_flow_sample(physics, plugins):
    import os
    env = dict(os.environ)
    env['PYTHONPATH'] = str(COMPANION/'python') + os.pathsep + env.get('PYTHONPATH','')
    result = subprocess.run([sys.executable, str(COMPANION/'samples/native_extensions.py'),
                             str(plugins('companion_spring').parent)],
                            check=True, capture_output=True, text=True, env=env)
    assert '"spring_body"' in result.stdout and '"ball"' in result.stdout


def test_non_cpu_mode_rejected_before_plugin_start(physics, plugins):
    import os
    env = dict(os.environ)
    env['PYTHONPATH'] = str(COMPANION/'python') + os.pathsep + env.get('PYTHONPATH','')
    code = '''
import ctypes as C, sys
from ovphysx import PhysX
from ovphysx_companion.plugins import NativePlugin
from ovphysx_companion.native import CompanionError, Status
library=C.CDLL(sys.argv[1])
library.companion_sample_count.restype=C.c_uint32
physics=PhysX()
try:
    try:
        NativePlugin(physics,sys.argv[1])
    except CompanionError as error:
        assert error.status==Status.UNAVAILABLE, error
    else:
        raise AssertionError('unqualified mode was accepted')
finally:
    physics.destroy()
assert library.companion_sample_count(3)==0
'''
    subprocess.run([sys.executable,'-c',code,str(plugins('companion_spring'))],
                   check=True,capture_output=True,text=True,env=env)


def test_module_outlives_last_core_in_fresh_process(physics, plugins):
    import os
    env = dict(os.environ)
    env['PYTHONPATH'] = str(COMPANION/'python') + os.pathsep + env.get('PYTHONPATH','')
    code = '''
import sys
from ovphysx import PhysX
from ovphysx_companion.plugins import NativePlugin
from ovphysx_companion.native import CompanionError, Status
PhysX.set_cpu_mode(True)
bootstrap=PhysX()
module=NativePlugin(bootstrap,sys.argv[1])
bootstrap.destroy()
peer=PhysX()
try:
    try:
        NativePlugin(peer,sys.argv[1])
    except CompanionError as error:
        assert error.status==Status.ERROR, error
    else:
        raise AssertionError('destroying bootstrap lost the live registration')
    module.close()
    with NativePlugin(peer,sys.argv[1]):
        pass
finally:
    module.close()
    peer.destroy()
'''
    subprocess.run([sys.executable,'-c',code,str(plugins('companion_spring'))],
                   check=True,capture_output=True,text=True,env=env)


@pytest.mark.parametrize('variant',['FLAGS','BLOCK'])
def test_joint_post_create_rejection_releases_once(physics,plugins,variant):
    library,count=counts(plugins('companion_bad_'+variant))
    before=[count(i) for i in range(7)]
    with NativePlugin(physics,plugins('companion_bad_'+variant)) as module:
        source=stage('joint-rollback-'+variant)
        try:
            with PreviewSession(physics,source,read_ordinal=1) as preview:
                preview.step(1/120)
                assert count(0)==before[0]+1 and count(1)==before[1]+1
                assert count(2)==before[2] # never admitted to solver preparation
            assert count(1)==before[1]+1 # detach must not release rejected state again
        finally:
            source.destroy()


def test_geometry_required_optional_callbacks_capabilities_and_bounds_rollback(physics,plugins):
    path=plugins('companion_ball_BAD_REQUIRED')
    library,count=counts(path)
    with pytest.raises(CompanionError) as error:
        NativePlugin(physics,path)
    assert error.value.status==Status.INVALID_ARGUMENT and count(3)==0
    for variant in ('NO_QUERIES','BAD_BOUNDS'):
        path=plugins('companion_ball_'+variant)
        library,count=counts(path)
        before=[count(i) for i in range(5)]
        with NativePlugin(physics,path) as module:
            source=stage('ball-'+variant)
            try:
                with PreviewSession(physics,source,read_ordinal=1) as preview:
                    preview.step(1/120)
                    if variant=='NO_QUERIES':
                        assert module.capabilities==frozenset({'geometry_cpu'})
                        for operation in ('raycast','overlap','sweep','debug','gpu','direct_gpu','sdf','mesh_merge','particles','deformables'):
                            with pytest.raises(CompanionError) as error:module.require(operation)
                            assert error.value.status==Status.UNAVAILABLE
                        from ovphysx_companion.queries import SceneQueries
                        with SceneQueries(preview) as queries:
                            assert not queries.raycast((5,5,0),(0,-1,0),4).any
                    else:
                        assert count(0)==before[0]+1 and count(1)==before[1]+1
                assert count(0)==before[0]+1 and count(1)==before[1]+1
            finally:
                source.destroy()
        with pytest.raises(CompanionError) as error:
            _=module.capabilities
        assert error.value.status==Status.NOT_FOUND


def test_capability_table_canaries_and_compiled_callback_contracts(physics,plugins):
    from ovphysx_companion.plugins import _CapabilitiesApi
    table=_CapabilitiesApi();C.memset(C.byref(table),0xa5,C.sizeof(table));before=bytes(table)
    acquire=physics._lib.ovphysx_companion_native_capabilities_acquire
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_CapabilitiesApi)];acquire.restype=C.c_uint32
    assert acquire(2,C.sizeof(table),C.byref(table))==Status.ABI_MISMATCH
    assert bytes(table)==before
    assert acquire(1,C.sizeof(table),C.byref(table))==Status.OK
    bits=C.c_uint32(0xa5)
    assert table.query(0,C.byref(bits))==Status.NOT_FOUND and bits.value==0xa5
    with NativePlugin(physics,plugins('companion_spring')) as joint:
        assert joint.capabilities==frozenset({'joint_cpu','joint_refresh'})
        for operation in ('gpu','direct_gpu','articulation_dofs','tensor_dofs','sdf','mesh_merge'):
            with pytest.raises(CompanionError) as error:joint.require(operation)
            assert error.value.status==Status.UNAVAILABLE
    with NativePlugin(physics,plugins('companion_ball')) as ball:
        assert ball.capabilities==frozenset({'geometry_cpu','raycast','overlap','sweep','debug'})
    for name in ('companion_callback_contract','companion_ball_contract'):
        binary=plugins('companion_spring').parent/(name+('.exe' if sys.platform=='win32' else ''))
        subprocess.run([str(binary)],check=True,capture_output=True,text=True)


def test_articulation_rejected_before_joint_callback(physics,plugins,tmp_path):
    import ovstage
    source_text=(COMPANION/'samples/scenes/inspection.usda').read_text()
    text=source_text[:source_text.rfind('}')]+'''
    def CompanionSpringJoint "ExternalLinkConstraint" {
        rel physics:body1 = </World/Robot/Tip>
        bool physics:excludeFromArticulation = true
        point3f physics:localPos0 = (1, 0, 4)
    }
}
'''
    path=tmp_path/'articulation_custom.usda';path.write_text(text)
    library,count=counts(plugins('companion_spring'))
    before=count(6)
    with NativePlugin(physics,plugins('companion_spring')):
        source=ovstage.Stage('custom-articulation-mode')
        try:
            ovstage.population.open_usd_with_desc(source,str(path),ordinal=1,time=0,
                descs=[population_description([BUNDLE])])
            source.advance_write_floor(ordinal=1).wait()
            with PreviewSession(physics,source,read_ordinal=1) as preview:
                preview.step(1/120)
                from ovphysx_companion.inspection import Inspector,ObjectKind
                with Inspector(preview) as inspector:
                    assert inspector.read(inspector.resolve('/World/Robot/Tip')).type=='link'
                assert count(6)==before
        finally:
            source.destroy()


def test_geometry_recreation_and_pending_retirement_stress(physics,plugins):
    import numpy as np
    import ovstage
    from ovphysx_companion.queries import SceneQueries
    library,count=counts(plugins('companion_ball'))
    before=[count(i) for i in range(5)]
    with NativePlugin(physics,plugins('companion_ball')) as module:
        source=stage('geometry-recreated')
        try:
            for iteration in range(6):
                ordinal=iteration+1
                radius=.5+iteration*.1
                if iteration:
                    with ovstage.PathDictionary(source) as dictionary:
                        with dictionary.create_path_list_from_strings(['/World/Ball']) as paths:
                            with source.query_from_path_list(paths) as query:
                                source.write_attribute(query,'companion:radius',ordinal,
                                    np.array([radius],dtype=np.float32),is_array=False).wait()
                    source.advance_write_floor(ordinal=ordinal).wait()
                else:
                    radius=.75
                with PreviewSession(physics,source,read_ordinal=ordinal) as preview:
                    preview.step(.001)
                    with SceneQueries(preview) as queries:
                        hit=queries.raycast((5,5,0),(0,-1,0),5)
                        assert hit.any and hit.hits[0].distance==pytest.approx(3-radius,abs=.01)
                    preview.simulate(.001)
                    with pytest.raises(CompanionError) as error:module.close()
                    assert error.value.status==Status.BUSY
                    preview.fetch()
                assert count(0)==before[0]+iteration+1
                assert count(1)==before[1]+iteration+1
        finally:
            source.destroy()
    assert count(4)==before[4]+1


def test_geometry_mass_optional_queries_and_debug_execute(physics,plugins):
    import math
    from ovphysx_companion.queries import SceneQueries,Geometry
    from ovphysx_companion.inspection import Inspector
    from ovphysx_companion.debug import DebugDraw,Parameter
    library,count=counts(plugins('companion_ball'))
    before=[count(i) for i in range(9)]
    with NativePlugin(physics,plugins('companion_ball')):
        source=stage('ball-query-debug')
        try:
            with PreviewSession(physics,source,read_ordinal=1) as preview:
                preview.step(.001)
                with Inspector(preview) as inspector:
                    mass=inspector.read(inspector.resolve('/World/Ball')).mass
                    assert mass==pytest.approx(1000*(4/3)*math.pi*.75**3,rel=1e-5)
                with SceneQueries(preview) as query:
                    assert query.raycast((5,5,0),(0,-1,0),4).any
                    assert query.overlap(Geometry.sphere(.1),(5,2,0)).any
                    assert query.sweep(Geometry.sphere(.1),(5,5,0),(0,-1,0),4).any
                with DebugDraw(preview) as debug:
                    debug.configure(Parameter.COLLISION_SHAPES)
                    preview.step(.001)
                    assert debug.capture().lines
                assert all(count(i)>before[i] for i in range(5,9))
        finally:
            source.destroy()


@pytest.mark.parametrize('collider_only,mixed',[(False,False),(False,True),(True,False)])
@pytest.mark.parametrize('scaled',[False,True])
def test_custom_geometry_point_instances_have_distinct_lifetime(physics,plugins,tmp_path,scaled,mixed,collider_only):
    import ovstage
    from ovphysx_companion.inspection import Inspector
    text='''#usda 1.0
(defaultPrim = "World")
def Xform "World" {
    def PhysicsScene "Scene" {
        float physics:gravityMagnitude = 0
    }
    def PointInstancer "Instances" {
        rel prototypes = </World/Instances/Ball>
        int[] protoIndices = [0,0]
        point3f[] positions = [(0,2,0),(4,2,0)]
        quath[] orientations = [(1,0,0,0),(1,0,0,0)]
        def Sphere "Ball" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "CompanionBallAPI"]) {
            float companion:radius = 0.5
        }
    }
}
'''
    if collider_only:
        text=text.replace('def PointInstancer "Instances" {',
                          'def PointInstancer "Instances" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {')
        text=text.replace('["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "CompanionBallAPI"]',
                          '["PhysicsCollisionAPI", "CompanionBallAPI"]')
    if mixed:
        text=text.replace('float companion:radius = 0.5','''float companion:radius = 0.5
            def Cube "Ordinary" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {
                double size = .25
                double3 xformOp:translate = (0,1,0)
                uniform token[] xformOpOrder = ["xformOp:translate"]
            }''')
    if scaled:
        text=text.replace('int[] protoIndices = [0,0]','int[] protoIndices = [0,0]\n        float3[] scales = [(2,2,2),(2,2,2)]')
    path=tmp_path/'custom_instances.usda';path.write_text(text)
    library,count=counts(plugins('companion_ball'))
    before=[count(i) for i in range(5)]
    with NativePlugin(physics,plugins('companion_ball')):
        source=ovstage.Stage('custom-point-instances')
        try:
            ovstage.population.open_usd_with_desc(source,str(path),ordinal=1,time=0,
                descs=[population_description([BUNDLE])])
            source.advance_write_floor(ordinal=1).wait()
            with PreviewSession(physics,source,read_ordinal=1) as preview,Inspector(preview) as inspector:
                preview.step(.001)
                if not collider_only:
                    a=inspector.resolve('/World/Instances/Ball',instance=0)
                    b=inspector.resolve('/World/Instances/Ball',instance=1)
                    assert a!=b
                    assert inspector.read(a).pose[:3]!=inspector.read(b).pose[:3]
                expected=0 if scaled else 2
                assert count(0)==before[0]+expected and count(1)==before[1]
            assert count(1)==before[1]+expected
        finally:
            source.destroy()


def test_joint_center_of_mass_edit_updates_local_solver_frame(physics,plugins):
    import numpy as np
    import ovstage
    library,count=counts(plugins('companion_spring'))
    library.companion_sample_last_com_x.restype=C.c_float
    with NativePlugin(physics,plugins('companion_spring')):
        source=stage('joint-com-frame')
        try:
            with PreviewSession(physics,source,read_ordinal=1) as preview:
                preview.step(.001)
                before=count(7)
                with ovstage.PathDictionary(source) as dictionary:
                    with dictionary.create_path_list_from_strings(['/World/Body']) as paths:
                        with source.query_from_path_list(paths) as query:
                            source.write_attribute(query,'physics:centerOfMass',2,
                                np.array([[.2,0,0]],dtype=np.float32),is_array=False).wait()
                source.advance_write_floor(ordinal=2).wait()
                preview.update(2,2);preview.step(.001)
                assert count(7)>before
                assert library.companion_sample_last_com_x()==pytest.approx(-.2)
        finally:
            source.destroy()


def test_object_deletion_releases_custom_state_before_attachment_close(physics,plugins):
    import ovstage
    spring_lib,spring=counts(plugins('companion_spring'))
    ball_lib,ball=counts(plugins('companion_ball'))
    before_s,before_b=[spring(i) for i in range(2)],[ball(i) for i in range(2)]
    with NativePlugin(physics,plugins('companion_spring')) as joint,NativePlugin(physics,plugins('companion_ball')) as geometry:
        source=stage('delete-custom-objects')
        try:
            with PreviewSession(physics,source,read_ordinal=1) as preview:
                preview.step(.001)
                with ovstage.PathDictionary(source) as dictionary:
                    with dictionary.create_path_list_from_strings(['/World/Spring','/World/Ball']) as paths:
                        with source.query_from_path_list(paths) as query:
                            source.delete_attributes(query,[],2).wait()
                source.advance_write_floor(ordinal=2).wait()
                preview.update(2,2);preview.step(.001)
                assert spring(0)==before_s[0]+1 and spring(1)==before_s[1]+1
                assert ball(0)==before_b[0]+1 and ball(1)==before_b[1]+1
                # Owner and an ordinary dynamic actor remain alive, but neither
                # deleted callback state may keep its module artificially Busy.
                joint.close();geometry.close()
                preview.step(.001)
            assert spring(1)==before_s[1]+1 and ball(1)==before_b[1]+1
        finally:
            source.destroy()


def test_registration_after_attach_refreshes_external_type_families(physics,plugins):
    library,count=counts(plugins('companion_spring'))
    before=[count(i) for i in range(2)]
    source=stage('late-custom-registration')
    try:
        with PreviewSession(physics,source,read_ordinal=1) as preview:
            preview.step(.001)
            assert count(0)==before[0]
            with NativePlugin(physics,plugins('companion_spring')) as joint:
                source.clone('/World/Spring',['/World/LateSpring'],2)
                source.advance_write_floor(ordinal=2).wait()
                preview.update(2,2);preview.step(.001)
                # The refreshed family scan finds the old prim and the new clone.
                assert count(0)==before[0]+2
                preview.close()
                assert count(1)==before[1]+2
    finally:source.destroy()


def test_authored_custom_clones_have_independent_parameters_and_state(physics,plugins):
    import ovstage
    import numpy as np
    import math
    from ovphysx_companion.inspection import Inspector
    spring_lib,spring=counts(plugins('companion_spring'))
    ball_lib,ball=counts(plugins('companion_ball'))
    before_s,before_b=[spring(i) for i in range(2)],[ball(i) for i in range(2)]
    spring_lib.companion_sample_last_refresh.restype=C.c_float
    with NativePlugin(physics,plugins('companion_spring')) as joint,NativePlugin(physics,plugins('companion_ball')):
        source=stage('cloned-custom-objects')
        try:
            source.clone('/World/Ball',['/World/CloneBall'],2)
            source.clone('/World/Spring',['/World/CloneSpring'],2)
            with ovstage.PathDictionary(source) as dictionary:
                for path,attr,value in (('/World/CloneBall','companion:radius',.4),
                                        ('/World/CloneSpring','companion:stiffness',250)):
                    with dictionary.create_path_list_from_strings([path]) as paths:
                        with source.query_from_path_list(paths) as query:
                            source.write_attribute(query,attr,3,np.array([value],dtype=np.float32),is_array=False).wait()
            source.advance_write_floor(ordinal=3).wait()
            with PreviewSession(physics,source,read_ordinal=3) as preview,Inspector(preview) as inspector:
                preview.step(.001)
                assert spring(0)==before_s[0]+2 and ball(0)==before_b[0]+2
                for path,radius in (('/World/Ball',.75),('/World/CloneBall',.4)):
                    assert inspector.read(inspector.resolve(path)).mass==pytest.approx(1000*4/3*math.pi*radius**3,rel=1e-5)
                for path,stiffness in (('/World/Spring',125),('/World/CloneSpring',250)):
                    joint.refresh_joint(preview.identity,path)
                    assert spring_lib.companion_sample_last_refresh()==stiffness
            assert spring(1)==before_s[1]+2 and ball(1)==before_b[1]+2
        finally:source.destroy()


def test_unregister_during_native_callback_is_busy_and_retains_code(physics,plugins):
    from concurrent.futures import ThreadPoolExecutor
    import time
    library,count=counts(plugins('companion_spring'))
    library.companion_sample_hold_prepare.argtypes=[C.c_bool]
    library.companion_sample_prepare_entered.restype=C.c_bool
    before=[count(i) for i in range(5)]
    with NativePlugin(physics,plugins('companion_spring')) as module:
        source=stage('callback-retirement')
        try:
            with PreviewSession(physics,source,read_ordinal=1) as preview,ThreadPoolExecutor(1) as pool:
                library.companion_sample_hold_prepare(True)
                future=pool.submit(preview.step,.001)
                try:
                    deadline=time.monotonic()+10
                    while not library.companion_sample_prepare_entered() and time.monotonic()<deadline:
                        time.sleep(.001)
                    assert library.companion_sample_prepare_entered()
                    with pytest.raises(CompanionError) as error:module.close()
                    assert error.value.status==Status.BUSY
                    assert count(4)==before[4]
                finally:
                    library.companion_sample_hold_prepare(False)
                future.result(timeout=10)
            assert count(0)==before[0]+1 and count(1)==before[1]+1
        finally:source.destroy()
    assert count(4)==before[4]+1


def test_static_mixed_custom_prototype_preserves_child_offsets(physics,plugins,tmp_path):
    import ovstage
    from ovphysx_companion.queries import SceneQueries
    path=tmp_path/'static-custom-compound.usda'
    path.write_text("""#usda 1.0
(metersPerUnit = 1)
def Xform "World" {
    def PhysicsScene "Scene" {
        float physics:gravityMagnitude = 0
    }
    def PointInstancer "Instances" {
        rel prototypes = </World/Instances/Prototype>
        int[] protoIndices = [0,0]
        point3f[] positions = [(0,0,0),(10,0,0)]
        def Xform "Prototype" {
            double3 xformOp:translate = (1,0,0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
            def Sphere "Custom" (prepend apiSchemas = ["PhysicsCollisionAPI","CompanionBallAPI"]) {
                float companion:radius = .5
            }
            def Cube "Ordinary" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {
                double size = .5
                double3 xformOp:translate = (0,2,0)
                uniform token[] xformOpOrder = ["xformOp:translate"]
            }
        }
    }
}
""")
    with NativePlugin(physics,plugins('companion_ball')):
        source=ovstage.Stage('static-custom-compound')
        try:
            ovstage.population.open_usd_with_desc(source,str(path),ordinal=1,time=0,descs=[population_description([BUNDLE])])
            source.advance_write_floor(ordinal=1).wait()
            with PreviewSession(physics,source,read_ordinal=1) as owner,SceneQueries(owner) as queries:
                owner.step(.001)
                for x in (1,11):
                    for y,height in ((0,.5),(2,.25)):
                        hit=queries.raycast((x,y,5),(0,0,-1),10).hits
                        assert len(hit)==1 and hit[0].position==pytest.approx((x,y,height))
        finally:source.destroy()
