# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-PREVIEW-001
# @covers AC-2 AC-3 AC-4
"""Customer-USD Inspector flow: snapshot, real worker simulation, commit and undo."""
from pathlib import Path
import sys
from pxr import Usd, Sdf
from ovphysx_companion.usd.session import EditSession
from ovphysx_companion.usd.preview import capture_preview, simulate_snapshot, commit_preview, PreviewDisplay


def run():
    source = Usd.Stage.Open(str(Path(__file__).parent / 'scenes/two_boxes.usda'))
    stage = Usd.Stage.Open(source.Flatten())
    layer = stage.GetRootLayer()
    display_layer=Sdf.Layer.CreateAnonymous('inspector-output')
    stage.GetSessionLayer().subLayerPaths.append(display_layer.identifier)
    with EditSession(stage) as edits:
        snapshot = capture_preview(edits, ['/World/BoxA'])
        poses = simulate_snapshot(edits, snapshot, python_executable=sys.executable, dt=1/60, steps=4)
        with PreviewDisplay(edits,layer=display_layer) as display:
            display.publish(snapshot,poses) # render without changing input revision
        assert display_layer.empty # cancel/Play removes only temporary output
        before = layer.ExportToString()
        patch = commit_preview(edits, snapshot, poses, layer=layer)
        patch.undo()
        assert layer.ExportToString() == before
        patch.redo()
        edits.invalidate()  # host Play / selection change invalidates previous preview work
    print('Inspector snapshot, native simulation, selected commit and undo passed')


if __name__ == '__main__':
    run()
