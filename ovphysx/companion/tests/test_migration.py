# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-MIGRATION-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-PYTHON-001
# @covers AC-3 AC-4
from pathlib import Path
import re
import subprocess
import sys
import os
import pytest
from ovphysx_companion.migration import authoring_helper,MigrationError,HELPERS


def test_ledger_has_all_interface_and_ux_dispositions_and_linked_contracts():
    root=Path(__file__).resolve().parents[1]
    text=(root/'plc/coverage.md').read_text()
    families=text.split('## Interface dispositions')[1].split('## UX extension dispositions')[0]
    extensions=text.split('## UX extension dispositions')[1].split('## Boundaries')[0]
    assert len(re.findall(r'^\| `I',families,re.M))==45
    assert len(re.findall(r'^\| `omni\.',extensions,re.M))==28
    for target in re.findall(r'\]\((requirements/[^)]+)\)',families+extensions):
        requirement=root/'plc'/target
        assert requirement.is_file()
        tests=re.findall(r'\]\((../../tests/[^)]+)\)',requirement.read_text())
        assert tests and all((requirement.parent/p).resolve().is_file() for p in tests)
    assert 'Identity-only' in families and 'No direct' in families


def test_shared_helper_resolution_and_explicit_migration_errors():
    from pxr import Usd,UsdPhysics
    for name in HELPERS:assert callable(authoring_helper(name))
    stage=Usd.Stage.CreateInMemory()
    authoring_helper('physicsUtils.add_rigid_box')(stage,'/Box')
    assert stage.GetPrimAtPath('/Box').HasAPI(UsdPhysics.RigidBodyAPI)
    for name in ('CreatePhysicsAttachmentCommand','deformableUtils.add_physx_deformable_body','unknown'):
        with pytest.raises(MigrationError,match='explicit'):authoring_helper(name)


def test_shared_authoring_never_initializes_runtime_in_fresh_process():
    script='''
import sys
from pxr import Usd
from ovphysx_companion.migration import authoring_helper
class BlockRuntime:
    def find_spec(self, name, *args):
        if name in ('ovphysx.api','ovphysx._bindings','ovstage'):
            raise AssertionError('runtime imported: '+name)
sys.meta_path.insert(0,BlockRuntime())
s=Usd.Stage.CreateInMemory()
authoring_helper('physicsUtils.add_box')(s,'/Box')
assert s.GetPrimAtPath('/Box')
assert 'ovstage' not in sys.modules
'''
    env=dict(os.environ,PYTHONPATH=str(Path(__file__).resolve().parents[1]/'python'))
    subprocess.run([sys.executable,'-c',script],env=env,check=True)


def test_unsupported_customer_usd_fails_before_edit(monkeypatch):
    from pxr import Usd
    from ovphysx_companion.usd import EditSession,DependencyError
    stage=Usd.Stage.CreateInMemory();before=stage.GetRootLayer().ExportToString()
    monkeypatch.delattr(Usd,'EditContext')
    with pytest.raises(DependencyError,match='Unsupported customer USD.*EditContext'):EditSession(stage)
    assert stage.GetRootLayer().ExportToString()==before
