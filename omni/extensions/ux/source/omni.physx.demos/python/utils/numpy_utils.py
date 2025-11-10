# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import numpy as np
import math


def quat_mul(a, b):
    assert a.shape == b.shape, f"Cannot multiply quat tensors with shape mismatch (a={a.shape}, b={b.shape})"
    shape = a.shape
    a = a.reshape(-1, 4)
    b = b.reshape(-1, 4)

    x1, y1, z1, w1 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    x2, y2, z2, w2 = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz
    qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
    w = qq - ww + (z1 - y1) * (y2 - z2)
    x = qq - xx + (x1 + w1) * (x2 + w2)
    y = qq - yy + (w1 - x1) * (y2 + z2)
    z = qq - zz + (z1 + y1) * (w2 - x2)

    quat = np.stack([x, y, z, w], axis=-1).reshape(shape)

    return quat


def normalize(x, eps: float = 1e-9):
    return x / np.clip(np.linalg.norm(x, axis=-1), a_min=eps, a_max=None)[:, None]


def quat_unit(a):
    return normalize(a)


def quat_from_angle_axis(angle, axis):
    theta = (angle / 2)[:, None]
    xyz = normalize(axis) * np.sin(theta)
    w = np.cos(theta)
    return quat_unit(np.concatenate([xyz, w], axis=-1))


def quat_rotate(q, v):
    shape = q.shape
    q_w = q[:, -1]
    q_vec = q[:, :3]
    a = v * (2.0 * q_w**2 - 1.0)[:, None]
    b = np.cross(q_vec, v) * q_w[:, None] * 2.0
    c = q_vec * np.sum(q_vec * v, axis=1).reshape(shape[0], -1) * 2.0
    return a + b + c


def quat_conjugate(a):
    shape = a.shape
    a = a.reshape(-1, 4)
    return np.concatenate((-a[:, :3], a[:, -1:]), axis=-1).reshape(shape)


def quat_axis(q, axis=0):
    basis_vec = np.zeros((q.shape[0], 3))
    basis_vec[:, axis] = 1
    return quat_rotate(q, basis_vec)


def get_angle_axis(q, eps: float = 1e-6):
    """returns axis angle representations of quaternions with the vector indicating axis of rotation and the
    magnitude of the vector gives to the rotation angle"""
    q = normalize([q])[0]
    half_angle = np.arccos(q[3])
    angle = 2 * half_angle
    denom = math.sqrt(1.0 - q[3] * q[3])
    if denom < eps:
        denom = 1.0
    return (q[:3] / denom) * angle


def orientation_error(desired, current):
    cc = quat_conjugate(current)
    q_r = quat_mul(desired, cc)
    if q_r[3] < 0.0:
        q_r = -q_r
    return get_angle_axis(q_r)


def get_x_rot_quat(angle_rad: float) -> np.array:
    """
    Get a quaternion that represents a rotation by angle around the x axis

    Inputs:
    angle_rad:      Rotation angle around x axis [radians]

    Returns: X-Rotation Quaternion  1x4 np array (qx, qy, qz, qw)
    """
    axis = np.array([[1, 0, 0]], dtype=np.float32)
    return quat_from_angle_axis(np.array([angle_rad], dtype=np.float32), axis)[0]


def get_y_rot_quat(angle_rad: float) -> np.array:
    """
    Get a quaternion that represents a rotation by angle around the y axis

    Inputs:
    angle_rad:      Rotation angle around y axis [radians]

    Returns: Y-Rotation Quaternion  1x4 np array (qx, qy, qz, qw)
    """
    axis = np.array([[0, 1, 0]], dtype=np.float32)
    return quat_from_angle_axis(np.array([angle_rad], dtype=np.float32), axis)[0]


def get_z_rot_quat(angle_rad: float) -> np.array:
    """
    Get a quaternion that represents a rotation by angle around the z axis

    Inputs:
    angle_rad:      Rotation angle around z axis [radians]

    Returns: Z-Rotation Quaternion  1x4 np array (qx, qy, qz, qw)
    """
    axis = np.array([[0, 0, 1]], dtype=np.float32)
    return quat_from_angle_axis(np.array([angle_rad], dtype=np.float32), axis)[0]
