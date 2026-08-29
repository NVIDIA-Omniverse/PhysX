# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Pure-Python validation tests for PhysXConfig.__post_init__.

No PhysX SDK instance is needed — these tests only exercise the dataclass
field type checks and carbonite_overrides conflict detection in config.py.
"""

import pytest
from ovphysx.config import _to_c_config

from ovphysx import PhysXConfig

# ---------------------------------------------------------------------------
# Default construction
# ---------------------------------------------------------------------------


def test_all_none_fields_valid():
    """PhysXConfig() with no arguments must not raise."""
    cfg = PhysXConfig()
    assert cfg.disable_contact_processing is None
    assert cfg.collision_cone_custom_geometry is None
    assert cfg.collision_cylinder_custom_geometry is None
    assert cfg.omnipvd_output_enabled is None
    assert cfg.num_threads is None
    assert cfg.scene_multi_gpu_mode is None
    assert cfg.omnipvd_ovd_recording_directory is None
    assert cfg.carbonite_overrides is None


# ---------------------------------------------------------------------------
# Bool fields
# ---------------------------------------------------------------------------

_BOOL_FIELDS = [
    "disable_contact_processing",
    "collision_cone_custom_geometry",
    "collision_cylinder_custom_geometry",
    "omnipvd_output_enabled",
]


@pytest.mark.parametrize("field", _BOOL_FIELDS)
@pytest.mark.parametrize("value", [True, False])
def test_bool_fields_accept_bool(field, value):
    """Each bool field must accept True and False without error."""
    cfg = PhysXConfig(**{field: value})
    assert getattr(cfg, field) == value


@pytest.mark.parametrize("field", _BOOL_FIELDS)
@pytest.mark.parametrize("bad_value", [1, 0, "true", "false", 1.0])
def test_bool_field_wrong_type_raises(field, bad_value):
    """Non-bool values for bool fields must raise TypeError."""
    with pytest.raises(TypeError):
        PhysXConfig(**{field: bad_value})


# ---------------------------------------------------------------------------
# Int fields
# ---------------------------------------------------------------------------

_INT_FIELDS = ["num_threads", "scene_multi_gpu_mode"]


@pytest.mark.parametrize(
    "field,value",
    [
        ("num_threads", 1),
        ("num_threads", 4),
        ("num_threads", 0),
        ("scene_multi_gpu_mode", 0),
        ("scene_multi_gpu_mode", 1),
        ("scene_multi_gpu_mode", 2),
    ],
)
def test_int_field_accepts_int(field, value):
    """Each int field must accept valid integer values."""
    cfg = PhysXConfig(**{field: value})
    assert getattr(cfg, field) == value


@pytest.mark.parametrize("field", _INT_FIELDS)
def test_int_field_rejects_bool(field):
    """Bool values for int fields must raise TypeError (bool is int subclass but excluded)."""
    with pytest.raises(TypeError):
        PhysXConfig(**{field: True})
    with pytest.raises(TypeError):
        PhysXConfig(**{field: False})


@pytest.mark.parametrize("field", _INT_FIELDS)
@pytest.mark.parametrize("bad_value", ["4", 1.5, None.__class__])
def test_int_field_rejects_str_and_float(field, bad_value):
    """String and float values for int fields must raise TypeError."""
    with pytest.raises(TypeError):
        PhysXConfig(**{field: bad_value})


# ---------------------------------------------------------------------------
# String fields
# ---------------------------------------------------------------------------


def test_str_field_accepts_str():
    """omnipvd_ovd_recording_directory accepts any string."""
    cfg = PhysXConfig(omnipvd_ovd_recording_directory="/tmp/pvd")
    assert cfg.omnipvd_ovd_recording_directory == "/tmp/pvd"


def test_str_field_accepts_empty_string():
    """Empty string is a valid directory value."""
    cfg = PhysXConfig(omnipvd_ovd_recording_directory="")
    assert cfg.omnipvd_ovd_recording_directory == ""


@pytest.mark.parametrize("bad_value", [42, True, 3.14, ["/tmp"]])
def test_str_field_rejects_non_str(bad_value):
    """Non-string values for str fields must raise TypeError."""
    with pytest.raises(TypeError):
        PhysXConfig(omnipvd_ovd_recording_directory=bad_value)


# ---------------------------------------------------------------------------
# carbonite_overrides
# ---------------------------------------------------------------------------


def test_carbonite_overrides_accepts_dict():
    """A dict value for carbonite_overrides must not raise."""
    cfg = PhysXConfig(carbonite_overrides={"/foo/bar": True})
    assert cfg.carbonite_overrides == {"/foo/bar": True}


def test_carbonite_overrides_accepts_empty_dict():
    """An empty dict is valid."""
    cfg = PhysXConfig(carbonite_overrides={})
    assert cfg.carbonite_overrides == {}


@pytest.mark.parametrize("bad_value", [["/foo"], "/foo", 42, True])
def test_carbonite_overrides_rejects_non_dict(bad_value):
    """Non-dict values for carbonite_overrides must raise TypeError."""
    with pytest.raises(TypeError):
        PhysXConfig(carbonite_overrides=bad_value)


# Known Carbonite paths that are mapped to typed fields — duplicating them
# in carbonite_overrides must raise ValueError at _to_c_config time.
_CONFLICTING_CARBONITE_PATHS = [
    "/physics/disableContactProcessing",
    "/physics/collisionConeCustomGeometry",
    "/physics/collisionCylinderCustomGeometry",
    "/physics/numThreads",
    "/physics/sceneMultiGPUMode",
    "/physics/omniPvdOutputEnabled",
    "/persistent/physics/omniPvdOvdRecordingDirectory",
]


@pytest.mark.parametrize("path", _CONFLICTING_CARBONITE_PATHS)
def test_carbonite_overrides_conflicts_typed_path(path):
    """A carbonite_overrides key that matches a typed field's Carbonite path must raise ValueError."""
    cfg = PhysXConfig(carbonite_overrides={path: "true"})
    with pytest.raises(ValueError, match="conflicts with typed field"):
        _to_c_config(cfg)


def test_carbonite_overrides_value_coercion():
    """carbonite_overrides values are coerced: bool→'true'/'false', int→str."""
    cfg = PhysXConfig(
        carbonite_overrides={
            "/custom/bool_key": True,
            "/custom/false_key": False,
            "/custom/int_key": 42,
            "/custom/float_key": 3.14,
        }
    )
    entries = _to_c_config(cfg)
    assert len(entries) == 4
    # No exception means coercion worked


def test_combined_typed_and_overrides():
    """Typed fields and non-conflicting carbonite_overrides can coexist."""
    cfg = PhysXConfig(
        num_threads=4,
        carbonite_overrides={"/custom/setting": "value"},
    )
    entries = _to_c_config(cfg)
    assert len(entries) == 2  # one typed + one carbonite
