# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Typed config for ovphysx.

Provides :class:`PhysXConfig`, a dataclass whose fields map 1:1 to the
C typed config enums in ``ovphysx_types.h``.  Only non-None fields are
applied; the rest keep their Carbonite/PhysX defaults.

Usage::

    from ovphysx import PhysX, PhysXConfig

    physx = PhysX(config=PhysXConfig(
        disable_contact_processing=True,
        num_threads=4,
        carbonite_overrides={"/physics/fabricUpdateVelocities": True},
    ))
"""

from __future__ import annotations

from dataclasses import dataclass


# Boolean config key enum values (must match ovphysx_config_bool_t)
_DISABLE_CONTACT_PROCESSING = 0
_COLLISION_CONE_CUSTOM_GEOMETRY = 1
_COLLISION_CYLINDER_CUSTOM_GEOMETRY = 2

# Int32 config key enum values (must match ovphysx_config_int32_t)
_NUM_THREADS = 0
_SCENE_MULTI_GPU_MODE = 1


# ---------------------------------------------------------------------------
# Low-level entry builders (used by _to_c_config and by set_config callers)
#
# Imports from ._bindings are deferred to first call so that importing
# PhysXConfig does not trigger native library loading (required for
# editable installs where setuptools reads __version__ at build time).
# ---------------------------------------------------------------------------

def _bindings():
    """Lazy import of native bindings (avoids loading libovphysx at import time)."""
    from . import _bindings as _b
    return _b


def _make_bool_entry(key: int, value: bool):
    _b = _bindings()
    entry = _b.ovphysx_config_entry_t()
    entry.key_type = _b.OVPHYSX_CONFIG_KEY_TYPE_BOOL
    entry.key.bool_key = key
    entry.value.bool_value = value
    return entry


def _make_int32_entry(key: int, value: int):
    _b = _bindings()
    entry = _b.ovphysx_config_entry_t()
    entry.key_type = _b.OVPHYSX_CONFIG_KEY_TYPE_INT32
    entry.key.int32_key = key
    entry.value.int32_value = value
    return entry


def _make_float_entry(key: int, value: float):
    _b = _bindings()
    entry = _b.ovphysx_config_entry_t()
    entry.key_type = _b.OVPHYSX_CONFIG_KEY_TYPE_FLOAT
    entry.key.float_key = key
    entry.value.float_value = value
    return entry


def _make_string_entry(key: int, value: str):
    _b = _bindings()
    entry = _b.ovphysx_config_entry_t()
    entry.key_type = _b.OVPHYSX_CONFIG_KEY_TYPE_STRING
    entry.key.string_key = key
    entry.value.string_value = _b.ovphysx_string_t(value)
    return entry


def _make_carbonite_entry(key: str, value: bool | int | float | str):
    """Build a carbonite escape-hatch config entry."""
    _b = _bindings()
    entry = _b.ovphysx_config_entry_t()
    entry.key_type = _b.OVPHYSX_CONFIG_KEY_TYPE_CARBONITE
    entry.key.carbonite_key = _b.ovphysx_string_t(key)
    if isinstance(value, bool):
        value_str = "true" if value else "false"
    else:
        value_str = str(value)
    entry.value.string_value = _b.ovphysx_string_t(value_str)
    return entry


# ---------------------------------------------------------------------------
# PhysXConfig dataclass
# ---------------------------------------------------------------------------

@dataclass
class PhysXConfig:
    """Typed configuration for ovphysx.

    All fields default to ``None`` (= use Carbonite/PhysX default).
    Only non-None fields are applied.

    Example::

        from ovphysx import PhysX, PhysXConfig

        physx = PhysX(config=PhysXConfig(
            disable_contact_processing=True,
            num_threads=4,
            carbonite_overrides={"/physics/fabricUpdateVelocities": True},
        ))
    """

    disable_contact_processing: bool | None = None
    collision_cone_custom_geometry: bool | None = None
    collision_cylinder_custom_geometry: bool | None = None
    num_threads: int | None = None
    scene_multi_gpu_mode: int | None = None  #: 0=disabled, 1=all GPUs, 2=skip first GPU
    carbonite_overrides: dict[str, bool | int | float | str] | None = None

    def __post_init__(self):
        _bool_fields = (
            "disable_contact_processing",
            "collision_cone_custom_geometry",
            "collision_cylinder_custom_geometry",
        )
        _int_fields = ("num_threads", "scene_multi_gpu_mode")
        for name in _bool_fields:
            value = getattr(self, name)
            if value is not None and not isinstance(value, bool):
                raise TypeError(f"PhysXConfig.{name} must be bool, got {type(value).__name__}")
        for name in _int_fields:
            value = getattr(self, name)
            if value is not None and (not isinstance(value, int) or isinstance(value, bool)):
                raise TypeError(f"PhysXConfig.{name} must be int, got {type(value).__name__}")
        if self.carbonite_overrides is not None and not isinstance(self.carbonite_overrides, dict):
            raise TypeError(
                f"PhysXConfig.carbonite_overrides must be dict, got {type(self.carbonite_overrides).__name__}"
            )


# ---------------------------------------------------------------------------
# Conversion to C config entries
# ---------------------------------------------------------------------------

# field_name -> (factory_fn, enum_key, carbonite_path)
_FIELD_TO_ENTRY: dict[str, tuple] = {
    "disable_contact_processing":        (_make_bool_entry,  _DISABLE_CONTACT_PROCESSING,        "/physics/disableContactProcessing"),
    "collision_cone_custom_geometry":     (_make_bool_entry,  _COLLISION_CONE_CUSTOM_GEOMETRY,     "/physics/collisionConeCustomGeometry"),
    "collision_cylinder_custom_geometry": (_make_bool_entry,  _COLLISION_CYLINDER_CUSTOM_GEOMETRY, "/physics/collisionCylinderCustomGeometry"),
    "num_threads":                       (_make_int32_entry, _NUM_THREADS,                        "/physics/numThreads"),
    "scene_multi_gpu_mode":              (_make_int32_entry, _SCENE_MULTI_GPU_MODE,               "/physics/sceneMultiGPUMode"),
}

# Reverse lookup: carbonite_path -> field_name (for conflict detection)
_KNOWN_CARBONITE_PATHS: dict[str, str] = {info[2]: name for name, info in _FIELD_TO_ENTRY.items()}



def _to_c_config(config: PhysXConfig) -> list[ovphysx_config_entry_t]:
    """Convert a PhysXConfig dataclass to a list of C config entries."""
    entries: list[ovphysx_config_entry_t] = []
    for field_name, (factory, key, _) in _FIELD_TO_ENTRY.items():
        value = getattr(config, field_name)
        if value is not None:
            entries.append(factory(key, value))
    if config.carbonite_overrides:
        for carb_key, carb_value in config.carbonite_overrides.items():
            if carb_key in _KNOWN_CARBONITE_PATHS:
                raise ValueError(
                    f"carbonite_overrides key '{carb_key}' conflicts with typed field "
                    f"'{_KNOWN_CARBONITE_PATHS[carb_key]}'. Use the typed field instead."
                )
            entries.append(_make_carbonite_entry(carb_key, carb_value))
    return entries
