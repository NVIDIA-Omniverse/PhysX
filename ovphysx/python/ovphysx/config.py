# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-1 AC-2
# @implements REQ-PYTHON-OMNIPVD-LATE-001
# @covers AC-1 AC-2
# @implements REQ-PYTHON-READPOOL-001
# @covers AC-2

"""Typed config for ovphysx.

@implements REQ-CAPI-NVTX-001
@covers AC-6

Provides :class:`PhysXConfig`, a dataclass whose fields map 1:1 to the
C typed config enums in ``ovphysx_types.h``.  Only non-None fields are
applied. The rest keep their Carbonite/PhysX defaults.

Usage::

    from ovphysx import PhysX, PhysXConfig

    physx = PhysX(config=PhysXConfig(
        disable_contact_processing=True,
        num_threads=4,
        carbonite_overrides={"/physics/updateToUsd": False},
    ))
"""

from __future__ import annotations

from dataclasses import dataclass


# Boolean config key enum values (must match ovphysx_config_bool_t)
_DISABLE_CONTACT_PROCESSING = 0
_COLLISION_CONE_CUSTOM_GEOMETRY = 1
_COLLISION_CYLINDER_CUSTOM_GEOMETRY = 2
_OMNIPVD_OUTPUT_ENABLED = 3
_NVTX_ENABLED = 4
_OMNIPVD_RECORDING_CAPABLE = 5

# Int32 config key enum values (must match ovphysx_config_int32_t)
_NUM_THREADS = 0
_SCENE_MULTI_GPU_MODE = 1
_OMNIPVD_TCP_PORT = 2
_OMNIPVD_TCP_TIMEOUT_MS = 3
_OVSTAGE_READ_POOL_MAX_MB = 4

# String config key enum values (must match ovphysx_config_string_t)
_OMNIPVD_OVD_RECORDING_DIRECTORY = 0
_COOKED_COLLIDER_CACHE_DIRECTORY = 1
_OMNIPVD_TRANSPORT = 2
_OMNIPVD_TCP_ADDRESS = 3


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

@dataclass(frozen=True)
class OmniPvdDestination:
    """Exact destination for a late OmniPVD recording session."""

    transport: str
    file_path: str = ""
    tcp_address: str = ""
    tcp_port: int = 0
    tcp_timeout_ms: int = 0

    @classmethod
    def file(cls, path: str) -> "OmniPvdDestination":
        """Record to the exact file path ``path``."""
        return cls(transport="file", file_path=path)

    @classmethod
    def tcp(cls, address: str, port: int, *, timeout_ms: int = 0) -> "OmniPvdDestination":
        """Connect to a ready TCP listener."""
        return cls(
            transport="tcp",
            tcp_address=address,
            tcp_port=port,
            tcp_timeout_ms=timeout_ms,
        )

    def __post_init__(self) -> None:
        for name in ("file_path", "tcp_address"):
            value = getattr(self, name)
            if not isinstance(value, str):
                raise TypeError(f"OmniPvdDestination.{name} must be str")
            if "\0" in value:
                raise ValueError(f"OmniPvdDestination.{name} must not contain NUL")
        for name in ("tcp_port", "tcp_timeout_ms"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool):
                raise TypeError(f"OmniPvdDestination.{name} must be int")

        if self.transport == "file":
            if not self.file_path or self.tcp_address or self.tcp_port != 0 or self.tcp_timeout_ms != 0:
                raise ValueError("FILE requires a non-empty file_path and empty/zero TCP fields")
        elif self.transport == "tcp":
            if self.file_path or not self.tcp_address or not 1 <= self.tcp_port <= 65535:
                raise ValueError("TCP requires an empty file_path, non-empty address, and port in 1..65535")
            if not 0 <= self.tcp_timeout_ms <= 2**31 - 1:
                raise ValueError("TCP timeout_ms must be in 0..INT32_MAX")
        else:
            raise ValueError("OmniPvdDestination.transport must be 'file' or 'tcp'")


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
            carbonite_overrides={"/physics/updateToUsd": False},
        ))
    """

    disable_contact_processing: bool | None = None
    collision_cone_custom_geometry: bool | None = None
    collision_cylinder_custom_geometry: bool | None = None
    num_threads: int | None = None
    scene_multi_gpu_mode: int | None = None  #: 0=disabled, 1=all GPUs, 2=skip first GPU. Used only when active_cuda_gpus is empty
    omnipvd_output_enabled: bool | None = None  #: Must be set before instance creation
    #: Enables late :meth:`PhysX.start_recording`. Set before the first instance is created.
    #: Process-wide and unset/false by default. ``omnipvd_output_enabled=True`` also
    #: enables this capability. Otherwise default creation installs no OmniPVD provider.
    omnipvd_recording_capable: bool | None = None
    omnipvd_ovd_recording_directory: str | None = None  #: Must be set before instance creation
    omnipvd_transport: str | None = None  #: Exact lowercase "file" or "tcp"
    omnipvd_tcp_address: str | None = None
    omnipvd_tcp_port: int | None = None
    omnipvd_tcp_timeout_ms: int | None = None  #: 0 uses the OS send timeout and a 3000 ms connect window
    #: Emit NVTX ranges for the ovphysx API calls and the PhysX SDK profile zones, for
    #: capture with Nsight Systems. Off by default. Must be set before instance creation.
    #: Setting OVPHYSX_NVTX=1 in the environment has the same effect.
    nvtx_enabled: bool | None = None
    #: Directory for the local cooked-collider (UJITSO) cache. Provide this to persist cooked
    #: colliders across runs and reuse them on the next launch. If left None, ovphysx cooks to a
    #: process-private temp directory that is discarded at shutdown, so nothing persists.
    #: Applied when the runtime first starts in a process. Later changes have no effect.
    cooked_collider_cache_dir: str | None = None
    #: Retention budget in MiB for the per-context device read-buffer pool that backs the ovstage
    #: output read (/physics/ovstageReadPoolMaxMB). Bounds the device and pinned-host memory the pool
    #: keeps between reads for reuse. ``0`` or any negative value DISABLES the pool (nothing is retained
    #: and every read allocates and frees as if the pool were absent). Default 256 when left None. Does not
    #: cap the memory a single read allocates, only what is retained.
    ovstage_read_pool_max_mb: int | None = None
    carbonite_overrides: dict[str, bool | int | float | str] | None = None

    def __post_init__(self):
        _bool_fields = (
            "disable_contact_processing",
            "collision_cone_custom_geometry",
            "collision_cylinder_custom_geometry",
            "omnipvd_output_enabled",
            "nvtx_enabled",
            "omnipvd_recording_capable",
        )
        _int_fields = (
            "num_threads",
            "scene_multi_gpu_mode",
            "omnipvd_tcp_port",
            "omnipvd_tcp_timeout_ms",
            "ovstage_read_pool_max_mb",
        )
        _str_fields = (
            "omnipvd_ovd_recording_directory",
            "omnipvd_transport",
            "omnipvd_tcp_address",
            "cooked_collider_cache_dir",
        )
        for name in _bool_fields:
            value = getattr(self, name)
            if value is not None and not isinstance(value, bool):
                raise TypeError(f"PhysXConfig.{name} must be bool, got {type(value).__name__}")
        for name in _int_fields:
            value = getattr(self, name)
            if value is not None and (not isinstance(value, int) or isinstance(value, bool)):
                raise TypeError(f"PhysXConfig.{name} must be int, got {type(value).__name__}")
        for name in _str_fields:
            value = getattr(self, name)
            if value is not None and not isinstance(value, str):
                raise TypeError(f"PhysXConfig.{name} must be str, got {type(value).__name__}")
        if self.carbonite_overrides is not None and not isinstance(self.carbonite_overrides, dict):
            raise TypeError(
                f"PhysXConfig.carbonite_overrides must be dict, got {type(self.carbonite_overrides).__name__}"
            )
        if self.omnipvd_transport is not None and self.omnipvd_transport not in ("file", "tcp"):
            raise ValueError("PhysXConfig.omnipvd_transport must be 'file' or 'tcp'")
        if self.omnipvd_transport == "tcp":
            if not self.omnipvd_tcp_address:
                raise ValueError("PhysXConfig.omnipvd_tcp_address must be non-empty for TCP")
            if self.omnipvd_tcp_port is None or not 1 <= self.omnipvd_tcp_port <= 65535:
                raise ValueError("PhysXConfig.omnipvd_tcp_port must be in 1..65535 for TCP")
            timeout = 0 if self.omnipvd_tcp_timeout_ms is None else self.omnipvd_tcp_timeout_ms
            if not 0 <= timeout <= 2**31 - 1:
                raise ValueError("PhysXConfig.omnipvd_tcp_timeout_ms must be in 0..INT32_MAX")


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
    "omnipvd_tcp_port":                  (_make_int32_entry, _OMNIPVD_TCP_PORT,                   "/physics/omniPvdTcpPort"),
    "omnipvd_tcp_timeout_ms":            (_make_int32_entry, _OMNIPVD_TCP_TIMEOUT_MS,             "/physics/omniPvdTcpTimeoutMs"),
    "ovstage_read_pool_max_mb":          (_make_int32_entry, _OVSTAGE_READ_POOL_MAX_MB,          "/physics/ovstageReadPoolMaxMB"),
    "omnipvd_output_enabled":            (_make_bool_entry,  _OMNIPVD_OUTPUT_ENABLED,             "/physics/omniPvdOutputEnabled"),
    "nvtx_enabled":                      (_make_bool_entry,  _NVTX_ENABLED,                       "/physics/nvtxEnabled"),
    "omnipvd_recording_capable":         (_make_bool_entry,  _OMNIPVD_RECORDING_CAPABLE,          "/physics/omniPvdRecordingCapable"),
    "omnipvd_ovd_recording_directory":   (_make_string_entry, _OMNIPVD_OVD_RECORDING_DIRECTORY,   "/persistent/physics/omniPvdOvdRecordingDirectory"),
    "omnipvd_transport":                 (_make_string_entry, _OMNIPVD_TRANSPORT,                 "/physics/omniPvdTransport"),
    "omnipvd_tcp_address":               (_make_string_entry, _OMNIPVD_TCP_ADDRESS,               "/physics/omniPvdTcpAddress"),
    "cooked_collider_cache_dir":         (_make_string_entry, _COOKED_COLLIDER_CACHE_DIRECTORY,   "/UJITSO/datastore/localCachePath"),
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
