# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Verify PEP 561 type stubs ship with the ovphysx Python package."""

# @implements REQ-PYTHON-LIFECYCLE-001
# @covers AC-1 AC-5
# @implements REQ-PYTHON-FRAME-001
# @covers AC-1
# @implements REQ-PYTHON-READ-001
# @covers AC-6
# @maps_to TEST-PYTHON-READ-001
# @implements REQ-CAPI-WRITE-001
# @covers AC-9
# @maps_to TEST-CAPI-WRITE-001
# @implements REQ-PYTHON-BINDING-DEVICE-001
# @covers AC-4
# @maps_to TEST-PYTHON-BINDING-DEVICE-001
# @implements REQ-PYTHON-UTILS-001
# @covers AC-2

from __future__ import annotations

from pathlib import Path

import pytest


@pytest.fixture
def pkg_dir() -> Path:
    """Directory of the installed/editable ovphysx package."""
    import ovphysx

    return Path(ovphysx.__file__).resolve().parent


def test_py_typed_marker_present(pkg_dir: Path):
    """ovphysx must ship py.typed so type checkers treat the package as typed."""
    assert (pkg_dir / "py.typed").is_file(), f"py.typed missing under {pkg_dir}"


def test_public_api_pyi_stubs_present(pkg_dir: Path):
    """Required public and utility stub files must be present."""
    expected = (
        "__init__.pyi",
        "api.pyi",
        "dlpack.pyi",
        "contact_types.pyi",
        "schemas.pyi",
        # ovphysx.utils is a package, so its stub lives inside it. Never as a
        # sibling utils.pyi: see test_utils_package_layout.py.
        "utils/simulation.pyi",
    )
    missing = [name for name in expected if not (pkg_dir / name).is_file()]
    assert not missing, f"Missing .pyi stubs under {pkg_dir}: {missing}"


def test_physx_stub_documents_constructor_params(pkg_dir: Path):
    """api.pyi should document the public PhysX constructor surface."""
    text = (pkg_dir / "api.pyi").read_text(encoding="utf-8")
    assert "class PhysX:" in text
    assert "config: PhysXConfig | None" in text
    assert "ignore_version_mismatch: bool" in text
    assert "active_cuda_gpus: str | None" in text
    assert "def destroy(self) -> None" in text
    physx_block = text.split("class PhysX:", 1)[1]
    assert "    def release(" not in physx_block
    assert "    def step_and_write_to_ovstage(" not in physx_block


def test_physx_stub_omits_main_object_context_manager(pkg_dir: Path):
    """The long-lived PhysX object must require explicit destruction."""
    text = (pkg_dir / "api.pyi").read_text(encoding="utf-8")
    physx_block = text.split("class PhysX:", 1)[1]
    assert "    def __enter__(" not in physx_block
    assert "    def __exit__(" not in physx_block


def test_read_group_arity_is_pinned():
    """`ReadGroup` is a public NamedTuple, so its field order and count are public shape.

    Adding a field breaks callers that unpack a whole group or compare one against a plain
    tuple. A deliberate change belongs in the changelog.
    """
    from ovphysx.api import ReadGroup

    assert ReadGroup._fields == (
        "attribute",
        "object_type",
        "ordinal",
        "is_array",
        "is_delete",
        "semantic",
        "prim_list",
        "prim_offset",
        "prim_count",
        "prim_index_map",
        "index_map",
        "layout_generation",
        "write_floor_ordinal",
        "tensors",
        "cuda_stream",
        "cuda_wait_event",
    )


def test_read_group_stub_matches_runtime(pkg_dir: Path):
    """The shipped stub must declare every runtime field, in order."""
    from ovphysx.api import ReadGroup

    text = (pkg_dir / "api.pyi").read_text(encoding="utf-8")
    assert "import warp as wp" in text
    body = text.split("class ReadGroup(NamedTuple):", 1)[1]
    body = body.split("\nclass ", 1)[0]
    declared = [line.strip().split(":", 1)[0] for line in body.splitlines() if line.startswith("    ") and ":" in line]
    assert declared == list(ReadGroup._fields)
    assert "    prim_index_map: wp.array | None" in body
    assert "    index_map: wp.array | None" in body
    assert "    tensors: list[wp.array]" in body


def test_write_group_arity_is_pinned():
    """The public write tuple keeps the runtime-owned fields in their shipped order."""
    from ovphysx.api import WriteGroup

    assert WriteGroup._fields == ("prim_list", "prim_offset", "prim_count", "tensors")


def test_write_surface_stub_matches_runtime(pkg_dir: Path):
    """The stub exposes the write context manager and uniform Warp tensor type."""
    from ovphysx.api import WriteGroup, WriteSession

    text = (pkg_dir / "api.pyi").read_text(encoding="utf-8")
    group_body = text.split("class WriteGroup(NamedTuple):", 1)[1].split("\nclass ", 1)[0]
    declared = [
        line.strip().split(":", 1)[0]
        for line in group_body.splitlines()
        if line.startswith("    ") and ":" in line
    ]
    assert declared == list(WriteGroup._fields)
    assert "    tensors: list[wp.array]" in group_body

    session_body = text.split("class WriteSession:", 1)[1].split("\nclass ", 1)[0]
    for method in ("commit", "close", "__enter__", "__exit__"):
        assert hasattr(WriteSession, method)
        assert f"    def {method}(" in session_body
    assert "    groups: list[WriteGroup]" in session_body

    physx_body = text.split("class PhysX:", 1)[1]
    write_signature = physx_body.split("    def write(", 1)[1].split("    def ", 1)[0]
    assert "object_type: SimObjectType" in write_signature
    assert "attribute_name: str" in write_signature
    assert ") -> WriteSession: ..." in write_signature


def test_tensor_binding_stub_declares_native_device(pkg_dir: Path):
    """The shipped TensorBinding stub exposes the runtime device property."""
    text = (pkg_dir / "api.pyi").read_text(encoding="utf-8")
    body = text.split("class TensorBinding:", 1)[1]
    body = body.split("\nclass ", 1)[0]
    assert "    def native_device(self) -> DLDevice" in body


def test_wheel_declares_warp_and_not_numpy():
    """AC-6: the wheel requires one Warp line and does not present NumPy as the read type.

    Warp pulls NumPy in transitively, so a direct numpy requirement would advertise an
    output-read contract this package does not have.
    """
    import tomllib

    pyproject = Path(__file__).resolve().parents[2] / "python" / "pyproject.toml"
    deps = tomllib.loads(pyproject.read_text(encoding="utf-8"))["project"]["dependencies"]
    names = [dep.split(">")[0].split("<")[0].split("=")[0].strip() for dep in deps]
    assert "warp-lang" in names, deps
    assert "numpy" not in names, deps
