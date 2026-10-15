# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-CAMERA-001
# @covers AC-1 AC-3 AC-4
from dataclasses import replace
import numpy as np
import pytest
from ovphysx_companion.camera import FollowCamera, DroneCamera, _rotation
from ovphysx_companion.output import BodyTarget, BodyOutput, capture_body_output
from ovphysx_companion.native import Identity


def test_follow_completed_core_body_output(attached):
    frames=capture_body_output(attached,source_revision='sealed-1',step=1,simulation_time=1/60,meters_per_unit=1)
    assert {frame.target.path for frame in frames}=={'/World/BoxA','/World/BoxB'}
    target=next(frame for frame in frames if frame.target.path=='/World/BoxA')
    camera=FollowCamera(target.target,offset=(0,-5,2),up=(0,0,1))
    pose=camera.update(target)
    np.testing.assert_allclose(pose.position,np.asarray(target.position)+(0,-5,2))
    copied=target.position
    attached.step(.1);attached.wait_all()
    newer=capture_body_output(attached,source_revision='sealed-1',step=2,simulation_time=1/60+.1,meters_per_unit=1)
    assert target.position==copied
    selected=next(frame for frame in newer if frame.target==target.target)
    assert selected.position[2]<target.position[2]
    assert camera.update(selected).position[2]<pose.position[2]


def test_follow_units_velocity_rotation_and_stale_incarnation():
    identity=BodyTarget(Identity(1,2),'/Body','revision-1')
    state=BodyOutput(identity,(100,0,0),(0,0,0,1),(200,0,0),1,.1,.01)
    camera=FollowCamera(identity,offset=(0,0,5),look_ahead=.5)
    pose=camera.update(state)
    np.testing.assert_allclose(pose.position,(2,0,5))
    np.testing.assert_allclose(_rotation(pose.orientation) @ (0,0,-1),(0,0,-1),atol=1e-7)
    assert camera.update(state) is None and camera.active
    assert camera.update(replace(state,step=0)) is None
    replacement=replace(state,target=replace(identity,source_revision='revision-2'),step=2)
    assert camera.update(replacement) is None and not camera.active
    assert camera.update(replace(state,step=3)) is None


def test_follow_invalid_input_preserves_last_pose_and_missing_disables():
    target=BodyTarget(Identity(1,2),'/Body','revision')
    state=BodyOutput(target,(0,0,0),(0,0,0,1),(0,0,0),1,0,1)
    camera=FollowCamera(target)
    pose=camera.update(state)
    with pytest.raises(ValueError):camera.update(replace(state,step=2,position=(float('nan'),0,0)))
    assert camera.pose==pose
    assert camera.update(None) is None and not camera.active


def test_drone_input_is_local_meters_and_stops_on_disable():
    camera=DroneCamera()
    pose=camera.update(velocity=(0,0,-2),dt=.5)
    np.testing.assert_allclose(pose.position,(0,0,-1),atol=1e-7)
    pose=camera.update(velocity=(0,0,-2),yaw_delta=np.pi/2,dt=.5)
    np.testing.assert_allclose(pose.position,(-1,0,-1),atol=1e-7)
    assert np.isclose(sum(v*v for v in pose.orientation),1)
    camera.disable()
    assert camera.update(velocity=(100,0,0),dt=1) is None
