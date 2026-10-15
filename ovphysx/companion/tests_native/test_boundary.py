# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-BOUNDARY-001
# @covers AC-1 AC-2 AC-3 AC-4
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import pytest

ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('companion',[False,True])
def test_isolated_core_simulation_with_package_absent_or_enabled(companion,tmp_path):
    # -I ignores inherited PYTHONPATH, cwd and user site. This core test venv has
    # no companion distribution; only the enabled process adds its source package.
    from ovphysx_companion.schemas import physx_schema_paths
    schema_paths = json.dumps([str(p) for p in physx_schema_paths()])
    script='''
import sys,importlib.util,json
from pathlib import Path
assert importlib.util.find_spec('ovphysx_companion') is None
if sys.argv[1]=='True':sys.path.insert(0,sys.argv[2])
import ovphysx,ovstage
from ovphysx import PhysX
from ovphysx.types import SimObjectType,ObjectScope
schemas=json.loads(sys.argv[4])
if ovphysx.newton_schema_root():schemas.append(str(ovphysx.newton_schema_root()))
ovstage.population.register_usd_schemas(schemas)
PhysX.set_cpu_mode(True)
p=PhysX();s=ovstage.Stage('core-boundary')
try:
    ovstage.population.open_usd(s,sys.argv[3],ordinal=1,domains=ovstage.PopulationDomain.ALL)
    s.advance_write_floor(ordinal=1).wait();p.wait_task(p.attach_ovstage(s,read_ordinal=1))
    p.wait_task(p.step(.01))
    with p.read_sync(SimObjectType.RIGID_BODY,['position'],scope=ObjectScope.ALL) as result:
        copied=[g.tensors[0].numpy().copy() for g in result.groups if g.tensors]
        assert copied and sum(len(a) for a in copied)>=2
    del result
    import gc
    gc.collect()
    if sys.argv[1]=='True':
        from ovphysx_companion.native import ToolsSession
        with ToolsSession(p) as tools:
            assert tools.identity.instance==p._omni_physx_sdk_handle.value
            assert tools.scene_statistics('/World/PhysicsScene').dynamic_bodies==2
    else:assert 'ovphysx_companion' not in sys.modules
    p.wait_task(p.detach_ovstage())
finally:p.destroy();s.destroy()
'''
    result=subprocess.run([sys.executable,'-I','-c',script,str(companion),str(ROOT/'python'),
                    str(ROOT/'samples/scenes/two_boxes.usda'),schema_paths],env=os.environ.copy(),cwd=tmp_path,
                   capture_output=True,text=True,timeout=30)
    assert result.returncode==0,result.stdout+result.stderr


def test_native_headers_sources_and_dependency_closure(physics):
    for directory in ('include','src'):
        for path in (ROOT/directory).rglob('*'):
            if path.suffix not in ('.h','.cpp'):continue
            includes=re.findall(r'^\s*#\s*include\s*[<"]([^>"\n]+)',path.read_text(),re.M)
            assert not any(name.startswith(('pxr/','omni/kit/')) for name in includes),path
    runtime=ROOT.parent/'ovruntime'
    for directory in ('include','source'):
        for path in (runtime/directory).rglob('*'):
            if path.suffix not in ('.h','.cpp'):continue
            assert not re.search(r'^\s*#\s*include\s*[<"]ovphysx_companion/',path.read_text(errors='replace'),re.M),path
    main=(ROOT.parent/'include/ovphysx/ovphysx.h').read_text()
    assert 'ovphysx_companion_' not in main
    if sys.platform=='linux':
        library=Path(physics._lib._name).resolve()
        dynamic=subprocess.run(['readelf','-d',str(library)],capture_output=True,text=True,check=True).stdout
        needed=[line.partition('[')[2].partition(']')[0] for line in dynamic.splitlines() if '(NEEDED)' in line and '[' in line]
        assert needed
        assert not any(re.search(r'lib(?:usd|sdf|tf|vt|gf|pxr|python|omni\.kit)',name,re.I) for name in needed)
