# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-DEFORM-WRITE-001
# @covers AC-1 AC-2 AC-3 AC-4

"""Deformable extension flow: preview completed output, bake frames, reject stale.

Frames contain known destination-space fixture values, not solver output. The
native producer must complete GPU work, map simulation vertices to the visual
mesh, convert frames/units and explicitly copy to CPU before entering this API.
"""

from contextlib import contextmanager

import numpy as np
from pxr import Sdf, Tf, Usd, UsdGeom

from ovphysx_companion.usd import EditSession, MeshFrame, StaleResultError, capture_mesh, write_simulation_output


class HostNoticePolicy:
    """Minimal host adapter: suppress output notices in the solver input observer."""

    def __init__(self, stage):
        self.publishing = False
        self.input_updates = 0
        self.notice = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self.changed, stage)

    def changed(self, notice, sender):
        if not self.publishing:
            self.input_updates += 1

    @contextmanager
    def output_guard(self):
        self.publishing = True
        try:
            yield
        finally:
            self.publishing = False


def run():
    stage = Usd.Stage.CreateInMemory()
    stage.SetTimeCodesPerSecond(60)
    mesh = UsdGeom.Mesh.Define(stage, "/Visual")
    rest = np.array([(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)], dtype=np.float32)
    mesh.CreatePointsAttr(rest.tolist())
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    bake = Sdf.Layer.CreateAnonymous("bake.usda")
    bake.timeCodesPerSecond = stage.GetTimeCodesPerSecond()
    stage.GetSessionLayer().subLayerPaths.append(bake.identifier)
    # Session layer is stronger than authored rest opinions; remove preview with undo.
    preview = stage.GetSessionLayer()
    host = HostNoticePolicy(stage)
    try:
        with EditSession(stage, output_guard=host.output_guard) as session:
            destination = capture_mesh(session, "/Visual")
            frame = MeshFrame(destination, rest + (0, 0, 0.25), step=1, simulation_time=1 / 60)
            patch = write_simulation_output(session, [frame], layer=preview, time_code=None)
            assert np.allclose(mesh.GetPointsAttr().Get(), frame.positions)
            patch.undo()  # Closing the preview invalidates outstanding work.
            destination = capture_mesh(session, "/Visual")
            for step in (1, 2):
                frame = MeshFrame(destination, rest + (0, 0, step * 0.25), step=step,
                                  simulation_time=step / 60)
                write_simulation_output(session, [frame], layer=bake, time_code=step)
                assert np.allclose(mesh.GetPointsAttr().Get(Usd.TimeCode(step)), frame.positions)
            assert host.input_updates == 0
            # Host selection/cancel/undo changes with no USD notice must invalidate explicitly.
            session.invalidate()
            try:
                write_simulation_output(session, [frame], layer=bake, time_code=3)
            except StaleResultError:
                pass
            else:
                raise AssertionError("stale result was accepted")
            assert bake.ListTimeSamplesForPath(Sdf.Path("/Visual.points")) == [1.0, 2.0]
    finally:
        host.notice.Revoke()
    return stage, bake


if __name__ == "__main__":
    _, bake = run()
    print(bake.ExportToString())
