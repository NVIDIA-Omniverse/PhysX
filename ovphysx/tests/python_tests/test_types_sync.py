# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Verify Python constants stay in sync with their C and C++ headers."""

import re
from pathlib import Path


def _repo_root() -> Path:
    """Walk up from this test file to find the ovphysx project root."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "include" / "ovphysx" / "ovphysx_types.h").exists():
            return parent
    raise RuntimeError("Cannot locate ovphysx project root (include/ovphysx/ovphysx_types.h)")


def _extract_c_tensor_values(header_text: str) -> set[int]:
    return set(int(v) for v in re.findall(r"OVPHYSX_TENSOR_\w+\s*=\s*(\d+)", header_text))


def _extract_c_api_status_values(header_text: str) -> set[int]:
    return set(int(v) for v in re.findall(r"OVPHYSX_API_\w+\s*=\s*(\d+)", header_text))


def _extract_c_log_level_values(header_text: str) -> set[int]:
    return set(int(v) for v in re.findall(r"OVPHYSX_LOG_\w+\s*=\s*(\d+)", header_text))


def _extract_cpp_max_tensor_rank(header_text: str) -> int:
    values = re.findall(r"\bconstexpr\s+int\s+kMaxDimensions\s*=\s*(\d+)\s*;", header_text)
    assert len(values) == 1, f"Expected one kMaxDimensions definition, found {len(values)}"
    return int(values[0])


def test_debug_render_scope_api_is_token_only():
    """Debug-render filtering must expose only the OVStage-native token API."""
    public_header = (_repo_root() / "include" / "ovphysx" / "ovphysx.h").read_text()
    runtime_header = (
        _repo_root() / "ovruntime" / "include" / "omni" / "physx" / "IPhysxVisualization.h"
    ).read_text()

    assert re.search(r"\bovphysx_debug_render_set_scope\s*\(", public_header) is None
    assert re.search(r"\bovphysx_intern_paths\s*\(", public_header) is None
    assert "ovphysx_debug_render_set_scope_tokens" in public_header
    assert re.search(r"\bsetVisualizationScope\s*\(", runtime_header) is None
    assert "setVisualizationScopeTokens" in runtime_header


def test_max_tensor_rank_matches_cpp_header():
    """The Python DLPack guard must accept the same ranks as TensorDesc."""
    from ovphysx._dlpack_utils import _MAX_TENSOR_RANK

    header = (
        _repo_root() / "ovruntime" / "include" / "omni" / "physics" / "tensors" / "TensorDesc.h"
    ).read_text()
    assert _MAX_TENSOR_RANK == _extract_cpp_max_tensor_rank(header)


def test_tensor_type_values_match_c_header():
    """Every OVPHYSX_TENSOR_* integer in the C header must appear in TensorType."""
    from ovphysx.types import TensorType

    header = (_repo_root() / "include" / "ovphysx" / "ovphysx_types.h").read_text()
    c_vals = _extract_c_tensor_values(header)
    py_vals = set(int(m) for m in TensorType)

    missing = c_vals - py_vals
    assert not missing, f"C header has tensor type values missing from TensorType: {missing}"


def test_api_status_values_match_c_header():
    """Every OVPHYSX_API_* integer in the C header must appear in ApiStatus."""
    from ovphysx.types import ApiStatus

    header = (_repo_root() / "include" / "ovphysx" / "ovphysx_types.h").read_text()
    c_vals = _extract_c_api_status_values(header)
    py_vals = set(int(m) for m in ApiStatus)

    missing = c_vals - py_vals
    assert not missing, f"C header has API status values missing from ApiStatus: {missing}"


def test_log_level_values_match_c_header():
    """Every OVPHYSX_LOG_* integer in the C header must appear in LogLevel."""
    from ovphysx.types import LogLevel

    header = (_repo_root() / "include" / "ovphysx" / "ovphysx_types.h").read_text()
    c_vals = _extract_c_log_level_values(header)
    py_vals = set(int(m) for m in LogLevel)

    missing = c_vals - py_vals
    assert not missing, f"C header has log level values missing from LogLevel: {missing}"


def test_no_python_only_tensor_values():
    """TensorType must not have values absent from the C header."""
    from ovphysx.types import TensorType

    header = (_repo_root() / "include" / "ovphysx" / "ovphysx_types.h").read_text()
    c_vals = _extract_c_tensor_values(header)
    py_vals = set(int(m) for m in TensorType if int(m) != 0)  # exclude INVALID

    extra = py_vals - c_vals
    assert not extra, f"TensorType has values not in C header: {extra}"


def test_tensor_type_no_duplicate_values():
    """All TensorType members must have unique integer values."""
    from ovphysx.types import TensorType

    values = [int(m) for m in TensorType]
    assert len(values) == len(set(values)), "TensorType has duplicate integer values"


def test_tensor_type_is_intenum():
    """TensorType members must be usable as plain ints (required by the C API)."""
    from ovphysx.types import TensorType

    assert isinstance(TensorType.ARTICULATION_ROOT_POSE, int)
    assert TensorType.ARTICULATION_ROOT_POSE == 10
    assert TensorType.ARTICULATION_DOF_POSITION + 1 == 31


def _extract_c_scene_query_mode_values(header_text: str) -> set[int]:
    return set(int(v) for v in re.findall(r"OVPHYSX_SCENE_QUERY_MODE_\w+\s*=\s*(\d+)", header_text))


def _extract_c_scene_query_geom_type_values(header_text: str) -> set[int]:
    return set(int(v) for v in re.findall(r"OVPHYSX_SCENE_QUERY_GEOMETRY_\w+\s*=\s*(\d+)", header_text))


def test_scene_query_mode_values_match_c_header():
    """Every OVPHYSX_SCENE_QUERY_MODE_* in C must appear in SceneQueryMode."""
    from ovphysx.types import SceneQueryMode

    header = (_repo_root() / "include" / "ovphysx" / "ovphysx_types.h").read_text()
    c_vals = _extract_c_scene_query_mode_values(header)
    py_vals = set(int(m) for m in SceneQueryMode)
    missing = c_vals - py_vals
    assert not missing, f"C header has SceneQueryMode values missing from Python: {missing}"
    extra = py_vals - c_vals
    assert not extra, f"SceneQueryMode has values not in C header: {extra}"


def test_scene_query_geometry_type_values_match_c_header():
    """Every OVPHYSX_SCENE_QUERY_GEOMETRY_* in C must appear in SceneQueryGeometryType."""
    from ovphysx.types import SceneQueryGeometryType

    header = (_repo_root() / "include" / "ovphysx" / "ovphysx_types.h").read_text()
    c_vals = _extract_c_scene_query_geom_type_values(header)
    py_vals = set(int(m) for m in SceneQueryGeometryType)
    missing = c_vals - py_vals
    assert not missing, f"C header has SceneQueryGeometryType values missing from Python: {missing}"
    extra = py_vals - c_vals
    assert not extra, f"SceneQueryGeometryType has values not in C header: {extra}"


def test_scene_query_hit_struct_size():
    """ctypes mirror of ovphysx_scene_query_hit_t must match the C layout (64 bytes)."""
    import ctypes

    from ovphysx._bindings import ovphysx_scene_query_hit_t

    assert ctypes.sizeof(ovphysx_scene_query_hit_t) == 64, (
        f"ovphysx_scene_query_hit_t size mismatch: " f"ctypes={ctypes.sizeof(ovphysx_scene_query_hit_t)}, expected=64"
    )
