# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-CAMERA-001
# @covers AC-1 AC-3 AC-4
"""Camera pose consumers in world meters; no native actors, USD or host input."""
from dataclasses import dataclass
import math
from .native import _host_sequence, _host_scalar


def _vector(value, width=3):
    value = tuple(_host_scalar(v) for v in _host_sequence(value))
    if len(value) != width or not all(math.isfinite(v) for v in value):
        raise ValueError(f'Expected {width} finite components')
    return value


def _rotation(quaternion):
    import numpy as np
    x, y, z, w = _vector(quaternion, 4)
    if abs(x*x + y*y + z*z + w*w - 1) > 1e-4:
        raise ValueError('Expected a unit xyzw quaternion')
    return np.array(((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                     (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                     (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y))))


def _look_at(eye, target, up):
    import numpy as np
    z = np.asarray(eye) - target  # Camera looks down local -Z, with local +Y up.
    if np.linalg.norm(z) < 1e-10:
        raise ValueError('Camera eye and target must be distinct')
    z /= np.linalg.norm(z)
    x = np.cross(up, z)
    if np.linalg.norm(x) < 1e-10:
        raise ValueError('Camera up is parallel to its viewing direction')
    x /= np.linalg.norm(x)
    matrix = np.column_stack((x, np.cross(z, x), z))
    # Stable eigenvector extraction handles both 180-degree and ordinary rotations.
    m = matrix
    k = np.array(((m[0,0]-m[1,1]-m[2,2], m[1,0]+m[0,1], m[2,0]+m[0,2], m[2,1]-m[1,2]),
                  (m[1,0]+m[0,1], m[1,1]-m[0,0]-m[2,2], m[2,1]+m[1,2], m[0,2]-m[2,0]),
                  (m[2,0]+m[0,2], m[2,1]+m[1,2], m[2,2]-m[0,0]-m[1,1], m[1,0]-m[0,1]),
                  (m[2,1]-m[1,2], m[0,2]-m[2,0], m[1,0]-m[0,1], np.trace(m)))) / 3
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    if q[3] < 0:
        q = -q
    return tuple(float(v) for v in q)


@dataclass(frozen=True)
class CameraPose:
    position: tuple
    orientation: tuple
    frame: str = 'world'
    units: str = 'meters'


class FollowCamera:
    """Follow exactly one BodyTarget using completed snapshots.

    Offsets are body-local meters; look-ahead uses world velocity in meters/sec.
    Missing/foreign incarnations disable the controller. Repeated or older steps
    hold the last pose without authoring a new frame. Re-enable by constructing a
    new controller after the host explicitly resolves the new target incarnation.
    """
    def __init__(self, target, *, offset=(0, 2, 5), aim_offset=(0, 0, 0), up=(0, 1, 0), look_ahead=0):
        from .output import BodyTarget
        if not isinstance(target, BodyTarget) or target.source_revision is None or not target.path.startswith('/'):
            raise ValueError('Expected an identified body target and source revision')
        self.target = target
        self.offset, self.aim_offset, self.up = _vector(offset), _vector(aim_offset), _vector(up)
        self.look_ahead = _host_scalar(look_ahead)
        if not math.isfinite(self.look_ahead) or self.look_ahead < 0:
            raise ValueError('look_ahead must be nonnegative seconds')
        self.active = True
        self._step, self._time = -1, -1.
        self.pose = None

    def disable(self):
        self.active = False

    def update(self, state):
        import numpy as np
        if not self.active:
            return None
        if state is None or state.target != self.target:
            self.disable()
            return None
        if (state.frame != 'world' or not math.isfinite(state.meters_per_unit) or state.meters_per_unit <= 0 or
                isinstance(state.step, bool) or not isinstance(state.step, int) or state.step < 0 or
                not math.isfinite(state.simulation_time) or state.simulation_time < 0):
            raise ValueError('Expected a completed world-space body frame with declared units')
        if state.step <= self._step or state.simulation_time < self._time:
            return None
        rotation = _rotation(state.orientation)
        position = np.asarray(_vector(state.position)) * state.meters_per_unit
        velocity = np.asarray(_vector(state.linear_velocity)) * state.meters_per_unit
        center = position + velocity * self.look_ahead
        eye, aim = center + rotation @ self.offset, center + rotation @ self.aim_offset
        pose = CameraPose(tuple(float(v) for v in eye), _look_at(eye, aim, self.up))
        self.pose, self._step, self._time = pose, state.step, state.simulation_time
        return pose


class DroneCamera:
    """Free-flight world-meter consumer of host input, independent of simulation.

    Input velocity is local meters/sec; yaw and pitch increments are radians.
    No keyboard, mouse, timeline or USD ownership is acquired.
    """
    def __init__(self, *, position=(0, 0, 0), yaw=0, pitch=0):
        self.position = _vector(position)
        self.yaw, self.pitch = _host_scalar(yaw), _host_scalar(pitch)
        if not math.isfinite(self.yaw) or not math.isfinite(self.pitch) or abs(self.pitch) >= math.pi/2:
            raise ValueError('Finite yaw and pitch strictly inside the vertical poles are required')
        self.active = True

    def disable(self):
        self.active = False

    def update(self, *, velocity=(0, 0, 0), yaw_delta=0, pitch_delta=0, dt):
        import numpy as np
        if not self.active:
            return None
        dt, yaw_delta, pitch_delta = map(_host_scalar, (dt, yaw_delta, pitch_delta))
        velocity = np.asarray(_vector(velocity))
        if not all(math.isfinite(v) for v in (dt, yaw_delta, pitch_delta)) or not 0 <= dt <= 1:
            raise ValueError('Supply finite input and dt in [0,1] seconds')
        yaw = math.remainder(self.yaw + yaw_delta, 2*math.pi)
        pitch = max(-math.pi/2+1e-5, min(math.pi/2-1e-5, self.pitch + pitch_delta))
        forward = np.array((-math.sin(yaw)*math.cos(pitch), math.sin(pitch), -math.cos(yaw)*math.cos(pitch)))
        orientation = _look_at(np.zeros(3), forward, (0,1,0))
        position = np.asarray(self.position) + (_rotation(orientation) @ velocity) * dt
        self.position, self.yaw, self.pitch = tuple(float(v) for v in position), yaw, pitch
        return CameraPose(self.position, orientation)
