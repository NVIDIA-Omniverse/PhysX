# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-NATIVE-SCHEMA-001
# @covers AC-1 AC-2
import json
import os
from pathlib import Path
import subprocess
import sys
import pytest

COMPANION = Path(__file__).resolve().parents[1]


def run(script):
    env = os.environ.copy()
    env['PYTHONPATH'] = os.pathsep.join([str(COMPANION/'python'),str(COMPANION/'samples'),env.get('PYTHONPATH','')])
    result = subprocess.run([sys.executable,'-c',script],text=True,capture_output=True,env=env,timeout=60)
    assert result.returncode == 0, result.stdout+result.stderr
    return result.stdout


def test_real_external_schemas_survive_independent_population():
    output = run('from external_schema_authoring import main; import json; print(json.dumps(main()))')
    assert json.loads(output.splitlines()[-1]) == {'/World/Spring.companion:stiffness':125.,
        '/World/Spring.companion:damping':2.,'/World/Ball.companion:radius':.75}


@pytest.mark.parametrize('case', ['late','parameter','kind'])
def test_external_schema_registration_fails_explicitly(case):
    script = '''
from external_schema_authoring import sample_bundle
from ovphysx_companion.schemas import SchemaBundle, ExternalSchema, register_customer_schemas
bundle=sample_bundle()
'''
    script += {
        'late': "from pxr import Usd; assert Usd.SchemaRegistry().FindConcretePrimDefinition('Cube')\nerror=RuntimeError\n",
        'parameter': "bundle=SchemaBundle(bundle.plugin,(ExternalSchema('CompanionSpringJoint','joint',('companion:unknown',)),))\nerror=ValueError\n",
        'kind': "bundle=SchemaBundle(bundle.plugin,(ExternalSchema('CompanionSpringJoint','geometry',('companion:stiffness',)),))\nerror=RuntimeError\n",
    }[case]
    script += '''
try:
    register_customer_schemas([bundle])
except error:
    pass
else:
    raise AssertionError('Malformed/late registration was accepted')
'''
    run(script)


def test_schema_helpers_import_without_usd_or_population():
    run("import sys; from ovphysx_companion.schemas import physx_schema_paths; "
        "assert len(physx_schema_paths()) == 2; "
        "assert 'pxr' not in sys.modules; assert 'ovstage' not in sys.modules; "
        "assert 'physx_usd_schemas' not in sys.modules")


def test_missing_physx_provider_reports_installation(monkeypatch):
    from ovphysx_companion import schemas

    def missing(name):
        raise schemas.metadata.PackageNotFoundError(name)

    monkeypatch.setattr(schemas.metadata, 'distribution', missing)
    with pytest.raises(FileNotFoundError, match='Install physx-usd-schemas'):
        schemas.physx_schema_paths()


@pytest.mark.parametrize('module', ['PhysxSchema', 'OmniUsdPhysicsDeformableSchema'])
@pytest.mark.parametrize('filename', ['plugInfo.json', 'generatedSchema.usda'])
def test_incomplete_physx_provider_reports_missing_resource(monkeypatch, tmp_path, module, filename):
    from types import SimpleNamespace
    from ovphysx_companion import schemas

    for name in ('PhysxSchema', 'OmniUsdPhysicsDeformableSchema'):
        resources = tmp_path / 'physx_usd_schemas' / name / 'resources'
        resources.mkdir(parents=True)
        for resource in ('plugInfo.json', 'generatedSchema.usda'):
            (resources / resource).write_text('{}')
    monkeypatch.setattr(schemas.metadata, 'distribution',
                        lambda name: SimpleNamespace(locate_file=lambda path: tmp_path / path))
    paths = schemas.physx_schema_paths()
    assert paths == [tmp_path / 'physx_usd_schemas' / name / 'resources'
                     for name in ('PhysxSchema', 'OmniUsdPhysicsDeformableSchema')]
    missing = tmp_path / 'physx_usd_schemas' / module / 'resources' / filename
    missing.unlink()
    with pytest.raises(FileNotFoundError, match='Incomplete physx-usd-schemas') as error:
        schemas.physx_schema_paths()
    assert str(missing) in str(error.value)
