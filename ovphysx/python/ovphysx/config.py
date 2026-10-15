# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-THREADS-001
# @covers AC-2

# @implements REQ-CAPI-OVD-NAMES-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-1 AC-2

# @implements REQ-PYTHON-OMNIPVD-LATE-001
# @covers AC-1 AC-2
# @implements REQ-PYTHON-READPOOL-001
# @covers AC-2

"""Typed config for ovphysx.

@implements REQ-CAPI-PROFILER-001
@covers AC-6

Provides :class:`PhysXConfig`, a dataclass whose fields map 1:1 to the
C typed config enums in ``ovphysx_types.h``.  Only non-None fields are
applied. The rest keep their Carbonite/PhysX defaults.

Usage::

    from ovphysx import PhysX, set_global_config
    from ovphysx.types import GlobalConfigKey

    set_global_config(GlobalConfigKey.DISABLE_CONTACT_PROCESSING, True)
    set_global_config(GlobalConfigKey.NUM_THREADS, 4)
    physx = PhysX()

``PhysXConfig`` is the legacy path. Carbonite-only settings still use
``carbonite_overrides``; passing ``config=`` emits a ``DeprecationWarning``.
"""

from __future__ import annotations

from dataclasses import dataclass
from inspect import Parameter, Signature, signature

from .types import ProfilerBackend


# Boolean config key enum values (must match ovphysx_config_bool_t)
_DISABLE_CONTACT_PROCESSING = 0
_COLLISION_CONE_CUSTOM_GEOMETRY = 1
_COLLISION_CYLINDER_CUSTOM_GEOMETRY = 2

# Int32 config key enum values (must match ovphysx_config_int32_t)
_NUM_THREADS = 0
_SCENE_MULTI_GPU_MODE = 1
_OVSTAGE_READ_POOL_MAX_MB = 4
_PROFILER_BACKEND = 5

# String config key enum values (must match ovphysx_config_string_t)
_COOKED_COLLIDER_CACHE_DIRECTORY = 1
_PROFILER_OUTPUT_PATH = 4

# The names accepted by PhysXConfig.profiler_backend, spelled the way the
# OVPHYSX_PROFILER environment variable spells them. ProfilerBackend is the single
# source of truth for the values; types is pure Python, so importing it here does not
# pull in the native bindings.
_PROFILER_BACKEND_NAMES = {member.name.lower(): int(member) for member in ProfilerBackend}

_RETIRED_OVD_FIELDS = frozenset({
    "omnipvd_recording_capable",
    "omnipvd_output_enabled",
    "omnipvd_ovd_recording_directory",
    "omnipvd_transport",
    "omnipvd_tcp_address",
    "omnipvd_tcp_port",
    "omnipvd_tcp_timeout_ms",
})


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
    """Exact destination for an explicit OVD recording session."""

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

    Settings with a ``GlobalConfigKey`` use the global API instead::

        from ovphysx import PhysX, set_global_config
        from ovphysx.types import GlobalConfigKey

        set_global_config(GlobalConfigKey.DISABLE_CONTACT_PROCESSING, True)
        set_global_config(GlobalConfigKey.NUM_THREADS, 4)
        physx = PhysX()

    Carbonite-only settings still use ``carbonite_overrides``; passing
    ``config=`` emits a ``DeprecationWarning``.
    """

    disable_contact_processing: bool | None = None
    collision_cone_custom_geometry: bool | None = None
    collision_cylinder_custom_geometry: bool | None = None
    num_threads: int | None = None
    scene_multi_gpu_mode: int | None = None  #: 0=disabled, 1=all GPUs, 2=skip first GPU. Used only when active_cuda_gpus is empty
    #: Profiler backend for the ovphysx API zones, the runtime's host-side zones (stage
    #: load, collision cooking, step orchestration) and the PhysX SDK profile zones.
    #: Accepts ``"none"``, ``"nvtx"`` or ``"trace"`` (case-insensitive), or a
    #: ``PROFILER_BACKEND_*`` value. Off by default; must be set before instance creation,
    #: because the profiler is bound during engine startup. Setting
    #: ``OVPHYSX_PROFILER=nvtx`` in the environment has the same effect.
    profiler_backend: str | int | None = None
    #: Destination file for ``profiler_backend="trace"``, ignored by the other backends.
    #: Unset writes ``ct-profile <date> <time>.json`` to the working directory. The trace
    #: is written when the last instance shuts down. Setting ``OVPHYSX_PROFILER_OUTPUT``
    #: in the environment has the same effect.
    profiler_output_path: str | None = None
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

    def __new__(cls, *args, **kwargs):
        retired = sorted(_RETIRED_OVD_FIELDS.intersection(kwargs))
        if retired:
            raise TypeError(
                f"PhysXConfig.{retired[0]} was removed in 0.7: call "
                "set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True) "
                "before the first PhysX(), attach, then "
                "PhysX.start_recording(OmniPvdDestination.file(...) or .tcp(...)). "
                "See docs/ovd_api_migration.md."
            )
        return super().__new__(cls)

    def __post_init__(self):
        _bool_fields = (
            "disable_contact_processing",
            "collision_cone_custom_geometry",
            "collision_cylinder_custom_geometry",
        )
        _int_fields = (
            "num_threads",
            "scene_multi_gpu_mode",
            "ovstage_read_pool_max_mb",
        )
        _str_fields = (
            "cooked_collider_cache_dir",
            "profiler_output_path",
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
        # Normalized to the int the C enum expects, so _to_c_config stays a plain lookup.
        # Names are accepted because that is how the environment variable spells it, and
        # a caller who writes profiler_backend="nvtx" should not have to import a constant.
        if self.profiler_backend is not None:
            if isinstance(self.profiler_backend, str):
                normalized = _PROFILER_BACKEND_NAMES.get(self.profiler_backend.lower())
                if normalized is None:
                    raise ValueError(
                        "PhysXConfig.profiler_backend must be one of "
                        f"{sorted(_PROFILER_BACKEND_NAMES)}, got {self.profiler_backend!r}"
                    )
                self.profiler_backend = normalized
            elif isinstance(self.profiler_backend, int) and not isinstance(self.profiler_backend, bool):
                if self.profiler_backend not in _PROFILER_BACKEND_NAMES.values():
                    raise ValueError(
                        "PhysXConfig.profiler_backend must be a PROFILER_BACKEND_* value, "
                        f"got {self.profiler_backend}"
                    )
            else:
                raise TypeError(
                    "PhysXConfig.profiler_backend must be str or int, "
                    f"got {type(self.profiler_backend).__name__}"
                )


class _ConfigConstructorSignature(Signature):
    """Report the nearest constructor without the diagnostic __new__ guard.

    Python 3.10 otherwise prefers inherited __new__ over a nearer __init__.
    A Signature also remains valid when functools.wraps copies the raw descriptor.
    As with any __signature__, inspect's eval_str/follow_wrapped flags cannot
    affect the constructor signature returned here.
    """

    __slots__ = ("_guard",)

    def __init__(self, *args, guard=None, **kwargs):
        super().__init__(*args, **kwargs)
        self._guard = guard

    def __get__(self, instance, owner):
        if instance is not None:
            # Callable instances must retain their __call__ signature.
            raise AttributeError("__signature__")
        if type(owner).__call__ is not type.__call__:
            # A metaclass's __call__ takes precedence over class constructors.
            raise AttributeError("__signature__")
        for cls in owner.__mro__:
            own_new = vars(cls).get("__new__")
            if own_new is not None and own_new is not self._guard:
                # Let inspect handle a subclass's own allocation constructor.
                break
            if "__init__" in vars(cls):
                try:
                    constructor = signature(cls.__init__)
                except NameError:
                    # Let inspect honor the caller's format for lazy annotations.
                    raise AttributeError("__signature__") from None
                parameters = list(constructor.parameters.values())
                if parameters and parameters[0].kind in (
                    Parameter.POSITIONAL_ONLY, Parameter.POSITIONAL_OR_KEYWORD,
                ):
                    parameters = parameters[1:]
                return constructor.replace(parameters=parameters)
        raise AttributeError("__signature__")


_base_constructor = signature(PhysXConfig.__init__)
PhysXConfig.__signature__ = _ConfigConstructorSignature(
    list(_base_constructor.parameters.values())[1:],
    return_annotation=_base_constructor.return_annotation,
    guard=vars(PhysXConfig)["__new__"],
)


# ---------------------------------------------------------------------------
# Conversion to C config entries
# ---------------------------------------------------------------------------

# field_name -> (factory_fn, enum_key, carbonite_path)
_FIELD_TO_ENTRY: dict[str, tuple] = {
    "disable_contact_processing":        (_make_bool_entry,  _DISABLE_CONTACT_PROCESSING,        "/physics/disableContactProcessing"),
    "collision_cone_custom_geometry":     (_make_bool_entry,  _COLLISION_CONE_CUSTOM_GEOMETRY,     "/physics/collisionApproximateCones"),
    "collision_cylinder_custom_geometry": (_make_bool_entry,  _COLLISION_CYLINDER_CUSTOM_GEOMETRY, "/physics/collisionApproximateCylinders"),
    "num_threads":                       (_make_int32_entry, _NUM_THREADS,                        "/persistent/physics/numThreads"),
    "scene_multi_gpu_mode":              (_make_int32_entry, _SCENE_MULTI_GPU_MODE,               "/physics/sceneMultiGPUMode"),
    "ovstage_read_pool_max_mb":          (_make_int32_entry, _OVSTAGE_READ_POOL_MAX_MB,          "/physics/ovstageReadPoolMaxMB"),
    "profiler_backend":                  (_make_int32_entry, _PROFILER_BACKEND,                   "/physics/profilerBackend"),
    "profiler_output_path":              (_make_string_entry, _PROFILER_OUTPUT_PATH,              "/physics/profilerOutputPath"),
    "cooked_collider_cache_dir":         (_make_string_entry, _COOKED_COLLIDER_CACHE_DIRECTORY,   "/UJITSO/datastore/localCachePath"),
}


def _to_c_config(config: PhysXConfig) -> list[ovphysx_config_entry_t]:
    """Convert a PhysXConfig dataclass to a list of C config entries."""
    entries: list[ovphysx_config_entry_t] = []
    known_paths = {info[2]: name for name, info in _FIELD_TO_ENTRY.items()}
    for field_name, (factory, key, _) in _FIELD_TO_ENTRY.items():
        value = getattr(config, field_name)
        if value is not None:
            entries.append(factory(key, value))
    if config.carbonite_overrides:
        for carb_key, carb_value in config.carbonite_overrides.items():
            if carb_key in known_paths:
                raise ValueError(
                    f"carbonite_overrides key '{carb_key}' conflicts with typed field "
                    f"'{known_paths[carb_key]}'. Use the typed field instead."
                )
            entries.append(_make_carbonite_entry(carb_key, carb_value))
    return entries
