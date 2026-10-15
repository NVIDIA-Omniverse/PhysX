# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-DEFORM-WRITE-001
# @covers AC-1 AC-2 AC-3
# @implements REQ-COMP-CAMERA-001
# @covers AC-1 AC-4
# @implements REQ-COMP-BUFFERS-001
# @covers AC-1 AC-2 AC-3
"""Transport-contract tests; actual deformable device simulation needs a GPU fixture."""
from contextlib import contextmanager
from types import SimpleNamespace
import numpy as np
import pytest
from ovphysx_companion.output import capture_deformable_output


def test_ragged_output_all_primitives_and_explicit_gpu_copy(attached, monkeypatch):
    import ovstage
    calls = []
    arrays = [np.ones((3,3), np.float32), np.full((4,3), 2, np.float32)]
    class Tensor:
        device = SimpleNamespace(is_cuda=True)
        def __init__(self, index):
            self.index = index
        def numpy(self):
            calls.append(self.index)
            return arrays[self.index]
    with ovstage.PathDictionary(attached._attached_ovstage) as dictionary:
        paths = dictionary.create_path_list_from_strings(['/World/SoftA', '/World/SoftB'])
        @contextmanager
        def read(*args, **kwargs):
            yield SimpleNamespace(groups=[SimpleNamespace(
                attribute=dictionary.intern_token(attribute), prim_list=paths, prim_count=2,
                is_array=True, is_delete=False, index_map=None, prim_index_map=None,
                tensors=(Tensor(0), Tensor(1))) for attribute in ('points', 'velocities')])
        monkeypatch.setattr(attached, 'read_sync', read)
        try:
            with pytest.raises(ValueError, match='copy_gpu'):
                capture_deformable_output(attached, kind='volume', step=1, simulation_time=.01)
            assert not calls
            output = capture_deformable_output(attached, kind='volume', step=1, simulation_time=.01, copy_gpu=True)
            assert calls == [0, 1, 0, 1]
            assert [record.path for record in output] == ['/World/SoftA', '/World/SoftB']
            assert [record.positions.shape for record in output] == [(3,3), (4,3)]
            assert output[0].owner == output[1].owner
            for array in arrays:
                array.fill(99)
            assert output[0].positions[0,0] == 1
            assert output[1].velocities[0,0] == 2
            assert not output[0].positions.flags.writeable
        finally:
            paths.release()


def test_scalar_body_output_reports_point_instance_scope_and_allows_filtered_read(physics):
    from pathlib import Path
    import ovstage
    from ovphysx_companion.output import capture_body_output
    stage=ovstage.Stage('mixed-output-scope')
    try:
        ovstage.population.open_usd(stage,str(Path(__file__).resolve().parents[1]/'samples/scenes/interaction.usda'),ordinal=1,domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait();physics.wait_task(physics.attach_ovstage(stage,read_ordinal=1));physics.step_sync(.01)
        arguments=dict(source_revision=1,step=1,simulation_time=.01,meters_per_unit=1)
        with pytest.raises(ValueError,match='scalar bodies only.*PointInstancer'):
            capture_body_output(physics,**arguments)
        output=capture_body_output(physics,pattern='/World/Body',**arguments)
        assert len(output)==1 and output[0].target.path=='/World/Body'
    finally:physics.wait_task(physics.detach_ovstage());stage.destroy()
