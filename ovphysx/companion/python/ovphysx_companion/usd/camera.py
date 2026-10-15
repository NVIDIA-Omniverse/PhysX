# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-CAMERA-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Explicit customer-USD camera output and restore policy."""
import math
from .session import _usd, StaleResultError
from ..camera import CameraPose, _vector, _rotation


class CameraOutput:
    """Publish world-meter poses to one explicitly selected local layer.

    With restore=True, this adapter owns that layer exclusively until close and
    restores its original contents. Conflicting layer edits are rejected rather
    than overwritten. Prefer a dedicated camera preview layer. Closing the edit
    session first runs this adapter's restoration; no camera input or simulation
    task is owned. The stage and all layer writes remain on the host's USD thread.
    """
    def __init__(self, session, path, *, layer, restore=True):
        Sdf, _, _, UsdGeom, _ = _usd()
        session._validate_layer(layer)
        path = Sdf.Path(path)
        prim = session.stage.GetPrimAtPath(path)
        if (not path.IsAbsolutePath() or not path.IsPrimPath() or not UsdGeom.Camera(prim) or
                not prim.IsActive() or prim.IsInstanceProxy() or prim.IsInPrototype()):
            raise ValueError('Select an existing editable camera prim')
        if not isinstance(restore, bool):
            raise TypeError('restore must be bool')
        self.session, self.path, self.layer, self.restore = session, str(path), layer, restore
        self._before = self._after = layer.ExportToString()
        self._closed = False
        self._remove_close = session.on_close(self.close)

    def publish(self, pose, *, source):
        """Apply one completed camera pose; output does not advance input revision."""
        if self._closed:
            raise RuntimeError('Camera output is closed')
        self.session.validate(source)
        _, _, Usd, UsdGeom, _ = _usd()
        from pxr import Gf
        if not isinstance(pose, CameraPose) or pose.frame != 'world' or pose.units != 'meters':
            raise ValueError('Camera pose must declare world meters')
        position = _vector(pose.position)
        _rotation(pose.orientation)
        prim = self.session.stage.GetPrimAtPath(self.path)
        if not prim or not UsdGeom.Camera(prim) or prim.IsInstanceProxy():
            self.close()
            raise ValueError('Camera destination was removed or replaced')
        if self.layer.ExportToString() != self._after:
            raise StaleResultError('Camera output layer changed outside this adapter')
        meters = UsdGeom.GetStageMetersPerUnit(self.session.stage)
        if not math.isfinite(meters) or meters <= 0:
            raise ValueError('Stage requires positive meters-per-unit')
        xform = UsdGeom.Xformable(prim)
        reset = xform.GetResetXformStack()
        parent = UsdGeom.XformCache().GetParentToWorldTransform(prim)
        if not reset and abs(parent.GetDeterminant()) < 1e-12:
            raise ValueError('Camera parent transform is singular')
        q = pose.orientation
        world = Gf.Matrix4d(1)
        world.SetRotate(Gf.Quatd(q[3], Gf.Vec3d(*q[:3])))
        world.SetTranslateOnly(Gf.Vec3d(*(v/meters for v in position)))
        matrix = world if reset else world * parent.GetInverse()
        def edit():
            name = 'xformOp:transform:companionCamera'
            attribute = prim.GetAttribute(name)
            operation = UsdGeom.XformOp(attribute) if attribute else xform.AddTransformOp(
                UsdGeom.XformOp.PrecisionDouble, 'companionCamera')
            if operation.GetOpType() != UsdGeom.XformOp.TypeTransform:
                raise ValueError('Incompatible camera output property')
            if not operation.Set(matrix) or not xform.SetXformOpOrder([operation], reset):
                raise RuntimeError('USD rejected camera output')
        self.session._apply(self.layer, edit, output=True)
        self._after = self.layer.ExportToString()

    def close(self):
        if self._closed:
            return
        self.session._check()
        if self.restore and self._before != self._after:
            if self.layer.ExportToString() != self._after:
                raise StaleResultError('Camera output layer changed; cannot restore unrelated edits')
            def restore():
                if not self.layer.ImportFromString(self._before):
                    raise RuntimeError('USD rejected camera restoration')
            self.session._apply(self.layer, restore, output=True)
        self._closed = True
        self._remove_close()

    def __enter__(self):
        if self._closed:
            raise RuntimeError('Camera output is closed')
        return self

    def __exit__(self, *args):
        self.close()


class FollowCameraOutput:
    """Host adapter coupling target revision checks, follow behavior and restoration.

    Target deletion, replacement, external input edits or missing state disable
    following and close the output. Repeat/older frames hold the previous pose.
    The host may reconstruct this adapter after deliberately resolving a target.
    """
    def __init__(self, session, path, controller, *, layer, restore=True):
        from ..camera import FollowCamera
        if not isinstance(controller, FollowCamera) or not controller.active:
            raise ValueError('An active follow controller is required')
        session.validate(controller.target.source_revision)
        self.controller = controller
        self.output = CameraOutput(session, path, layer=layer, restore=restore)
        self._closed = False
        self._remove_close = session.on_close(self.close)

    def update(self, state):
        if self._closed:
            return False
        if self.output.session.source != self.controller.target.source_revision:
            self.close()
            return False
        pose = self.controller.update(state)
        if not self.controller.active:
            self.close()
            return False
        if pose is None:
            return False
        self.output.publish(pose, source=self.controller.target.source_revision)
        return True

    def close(self):
        self.controller.disable()
        if self._closed:
            return
        self.output.close()
        self._closed = True
        self._remove_close()

    def __enter__(self):
        if self._closed:
            raise RuntimeError('Follow camera is closed')
        return self

    def __exit__(self, *args):
        self.close()
