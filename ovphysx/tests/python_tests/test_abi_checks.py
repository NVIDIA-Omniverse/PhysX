# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

import ctypes
from ctypes import c_int, c_uint64

from ovphysx._bindings import (
    DLDataType,
    DLDevice,
    DLTensor,
    ovphysx_config_entry_t,
    ovphysx_create_args,
)
from ovphysx.contact_types import ContactEventHeader, ContactPoint, FrictionAnchor

from ovphysx import _bindings as physx_bindings


def test_create_args_layout_matches():
    """Verify OmniCreateArgs structure layout matches the C library."""
    sizeof_fn = getattr(physx_bindings._lib, "ovphysx_internal_create_args_sizeof", None)
    offset_fn = getattr(physx_bindings._lib, "ovphysx_internal_create_args_offset", None)
    assert sizeof_fn is not None, "Missing ABI check function: ovphysx_internal_create_args_sizeof"
    assert offset_fn is not None, "Missing ABI check function: ovphysx_internal_create_args_offset"

    sizeof_fn.restype = c_uint64
    sizeof_fn.argtypes = []
    offset_fn.restype = c_uint64
    offset_fn.argtypes = [c_int]

    so_size = int(sizeof_fn())
    py_size = ctypes.sizeof(ovphysx_create_args)
    assert so_size == py_size, f"Structure size mismatch: C={so_size}, Python={py_size}"

    for idx, (name, _ctype) in enumerate(ovphysx_create_args._fields_):
        so_off = int(offset_fn(idx))
        py_off = getattr(ovphysx_create_args, name).offset
        assert so_off == py_off, f"Field {name} offset mismatch: C={so_off}, Python={py_off}"


def test_config_entry_layout_matches():
    """Verify ovphysx_config_entry_t structure layout matches the C library."""
    sizeof_fn = getattr(physx_bindings._lib, "ovphysx_internal_config_entry_sizeof", None)
    alignof_fn = getattr(physx_bindings._lib, "ovphysx_internal_config_entry_alignof", None)
    offset_fn = getattr(physx_bindings._lib, "ovphysx_internal_config_entry_offset", None)
    assert sizeof_fn is not None, "Missing ABI check function: ovphysx_internal_config_entry_sizeof"
    assert alignof_fn is not None, "Missing ABI check function: ovphysx_internal_config_entry_alignof"
    assert offset_fn is not None, "Missing ABI check function: ovphysx_internal_config_entry_offset"

    sizeof_fn.restype = c_uint64
    sizeof_fn.argtypes = []
    alignof_fn.restype = c_uint64
    alignof_fn.argtypes = []
    offset_fn.restype = c_uint64
    offset_fn.argtypes = [c_int]

    c_size = int(sizeof_fn())
    py_size = ctypes.sizeof(ovphysx_config_entry_t)
    assert c_size == py_size, f"config_entry_t size mismatch: C={c_size}, Python={py_size}"

    c_align = int(alignof_fn())
    py_align = ctypes.alignment(ovphysx_config_entry_t)
    assert c_align == py_align, f"config_entry_t alignment mismatch: C={c_align}, Python={py_align}"

    for idx, (name, _ctype) in enumerate(ovphysx_config_entry_t._fields_):
        c_off = int(offset_fn(idx))
        py_off = getattr(ovphysx_config_entry_t, name).offset
        assert c_off == py_off, f"config_entry_t field {name} offset mismatch: C={c_off}, Python={py_off}"


def test_dl_data_type_layout_matches():
    """Verify DLDataType structure layout matches the C library."""
    sizeof_fn = getattr(physx_bindings._lib, "ovphysx_internal_dl_data_type_sizeof", None)
    alignof_fn = getattr(physx_bindings._lib, "ovphysx_internal_dl_data_type_alignof", None)
    offset_fn = getattr(physx_bindings._lib, "ovphysx_internal_dl_data_type_offset", None)
    assert sizeof_fn is not None, "Missing ABI check function: ovphysx_internal_dl_data_type_sizeof"
    assert alignof_fn is not None, "Missing ABI check function: ovphysx_internal_dl_data_type_alignof"
    assert offset_fn is not None, "Missing ABI check function: ovphysx_internal_dl_data_type_offset"

    sizeof_fn.restype = c_uint64
    sizeof_fn.argtypes = []
    alignof_fn.restype = c_uint64
    alignof_fn.argtypes = []
    offset_fn.restype = c_uint64
    offset_fn.argtypes = [c_int]

    c_size = int(sizeof_fn())
    py_size = ctypes.sizeof(DLDataType)
    assert c_size == py_size, f"DLDataType size mismatch: C={c_size}, Python={py_size}"

    c_align = int(alignof_fn())
    py_align = ctypes.alignment(DLDataType)
    assert c_align == py_align, f"DLDataType alignment mismatch: C={c_align}, Python={py_align}"

    for idx, (name, _ctype) in enumerate(DLDataType._fields_):
        c_off = int(offset_fn(idx))
        py_off = getattr(DLDataType, name).offset
        assert c_off == py_off, f"DLDataType field {name} offset mismatch: C={c_off}, Python={py_off}"


def test_dl_device_layout_matches():
    """Verify DLDevice structure layout matches the C library."""
    sizeof_fn = getattr(physx_bindings._lib, "ovphysx_internal_dl_device_sizeof", None)
    alignof_fn = getattr(physx_bindings._lib, "ovphysx_internal_dl_device_alignof", None)
    offset_fn = getattr(physx_bindings._lib, "ovphysx_internal_dl_device_offset", None)
    assert sizeof_fn is not None, "Missing ABI check function: ovphysx_internal_dl_device_sizeof"
    assert alignof_fn is not None, "Missing ABI check function: ovphysx_internal_dl_device_alignof"
    assert offset_fn is not None, "Missing ABI check function: ovphysx_internal_dl_device_offset"

    sizeof_fn.restype = c_uint64
    sizeof_fn.argtypes = []
    alignof_fn.restype = c_uint64
    alignof_fn.argtypes = []
    offset_fn.restype = c_uint64
    offset_fn.argtypes = [c_int]

    c_size = int(sizeof_fn())
    py_size = ctypes.sizeof(DLDevice)
    assert c_size == py_size, f"DLDevice size mismatch: C={c_size}, Python={py_size}"

    c_align = int(alignof_fn())
    py_align = ctypes.alignment(DLDevice)
    assert c_align == py_align, f"DLDevice alignment mismatch: C={c_align}, Python={py_align}"

    for idx, (name, _ctype) in enumerate(DLDevice._fields_):
        c_off = int(offset_fn(idx))
        py_off = getattr(DLDevice, name).offset
        assert c_off == py_off, f"DLDevice field {name} offset mismatch: C={c_off}, Python={py_off}"


def test_dl_tensor_layout_matches():
    """Verify DLTensor structure layout matches the C library."""
    sizeof_fn = getattr(physx_bindings._lib, "ovphysx_internal_dl_tensor_sizeof", None)
    alignof_fn = getattr(physx_bindings._lib, "ovphysx_internal_dl_tensor_alignof", None)
    offset_fn = getattr(physx_bindings._lib, "ovphysx_internal_dl_tensor_offset", None)
    assert sizeof_fn is not None, "Missing ABI check function: ovphysx_internal_dl_tensor_sizeof"
    assert alignof_fn is not None, "Missing ABI check function: ovphysx_internal_dl_tensor_alignof"
    assert offset_fn is not None, "Missing ABI check function: ovphysx_internal_dl_tensor_offset"

    sizeof_fn.restype = c_uint64
    sizeof_fn.argtypes = []
    alignof_fn.restype = c_uint64
    alignof_fn.argtypes = []
    offset_fn.restype = c_uint64
    offset_fn.argtypes = [c_int]

    c_size = int(sizeof_fn())
    py_size = ctypes.sizeof(DLTensor)
    assert c_size == py_size, f"DLTensor size mismatch: C={c_size}, Python={py_size}"

    c_align = int(alignof_fn())
    py_align = ctypes.alignment(DLTensor)
    assert c_align == py_align, f"DLTensor alignment mismatch: C={c_align}, Python={py_align}"

    for idx, (name, _ctype) in enumerate(DLTensor._fields_):
        c_off = int(offset_fn(idx))
        py_off = getattr(DLTensor, name).offset
        assert c_off == py_off, f"DLTensor field {name} offset mismatch: C={c_off}, Python={py_off}"


def _check_struct_layout(label, struct, sizeof_name, alignof_name, offset_name):
    """Helper: cross-check a Python ctypes struct against C sizeof/alignof/offsetof exports."""
    sizeof_fn = getattr(physx_bindings._lib, sizeof_name, None)
    alignof_fn = getattr(physx_bindings._lib, alignof_name, None)
    offset_fn = getattr(physx_bindings._lib, offset_name, None)
    assert sizeof_fn is not None, f"Missing ABI check function: {sizeof_name}"
    assert alignof_fn is not None, f"Missing ABI check function: {alignof_name}"
    assert offset_fn is not None, f"Missing ABI check function: {offset_name}"

    sizeof_fn.restype = c_uint64
    sizeof_fn.argtypes = []
    alignof_fn.restype = c_uint64
    alignof_fn.argtypes = []
    offset_fn.restype = c_uint64
    offset_fn.argtypes = [c_int]

    c_size = int(sizeof_fn())
    py_size = ctypes.sizeof(struct)
    assert c_size == py_size, f"{label} size mismatch: C={c_size}, Python={py_size}"

    c_align = int(alignof_fn())
    py_align = ctypes.alignment(struct)
    assert c_align == py_align, f"{label} alignment mismatch: C={c_align}, Python={py_align}"

    for idx, (name, _ctype) in enumerate(struct._fields_):
        c_off = int(offset_fn(idx))
        py_off = getattr(struct, name).offset
        assert c_off == py_off, f"{label} field {name} offset mismatch: C={c_off}, Python={py_off}"


def test_contact_event_header_layout_matches():
    """Verify ContactEventHeader matches ovphysx_contact_event_header_t."""
    _check_struct_layout(
        "ContactEventHeader",
        ContactEventHeader,
        "ovphysx_internal_contact_event_header_sizeof",
        "ovphysx_internal_contact_event_header_alignof",
        "ovphysx_internal_contact_event_header_offset",
    )


def test_contact_point_layout_matches():
    """Verify ContactPoint matches ovphysx_contact_point_t."""
    _check_struct_layout(
        "ContactPoint",
        ContactPoint,
        "ovphysx_internal_contact_point_sizeof",
        "ovphysx_internal_contact_point_alignof",
        "ovphysx_internal_contact_point_offset",
    )


def test_friction_anchor_layout_matches():
    """Verify FrictionAnchor matches ovphysx_friction_anchor_t."""
    _check_struct_layout(
        "FrictionAnchor",
        FrictionAnchor,
        "ovphysx_internal_friction_anchor_sizeof",
        "ovphysx_internal_friction_anchor_alignof",
        "ovphysx_internal_friction_anchor_offset",
    )
