# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-THREADS-001
# @covers AC-2
#
# @implements REQ-PYTHON-OMNIPVD-LATE-001
# @covers AC-1

"""Pure-Python configuration validation and migration diagnostics.

@implements REQ-CAPI-PROFILER-001
@covers AC-6

No PhysX SDK instance is needed. These tests exercise dataclass construction,
field type checks and carbonite_overrides conflict detection in config.py.
"""

import abc
from copy import copy, deepcopy
from dataclasses import dataclass, replace
from functools import wraps
import inspect
import os
import pickle
import subprocess
import sys
import textwrap

import pytest
from ovphysx.config import _to_c_config

from ovphysx import OmniPvdDestination, PhysXConfig, ProfilerBackend

# ---------------------------------------------------------------------------
# Default construction
# ---------------------------------------------------------------------------


def test_all_none_fields_valid():
    """PhysXConfig() with no arguments must not raise."""
    cfg = PhysXConfig()
    assert cfg.disable_contact_processing is None
    assert cfg.collision_cone_custom_geometry is None
    assert cfg.collision_cylinder_custom_geometry is None
    assert cfg.profiler_backend is None
    assert cfg.profiler_output_path is None
    assert cfg.num_threads is None
    assert cfg.scene_multi_gpu_mode is None
    assert cfg.ovstage_read_pool_max_mb is None
    assert cfg.carbonite_overrides is None


# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-2
@pytest.mark.parametrize(
    "field,value",
    [
        ("omnipvd_recording_capable", True),
        ("omnipvd_output_enabled", True),
        ("omnipvd_ovd_recording_directory", "captures"),
        ("omnipvd_transport", "tcp"),
        ("omnipvd_tcp_address", "127.0.0.1"),
        ("omnipvd_tcp_port", 5425),
        ("omnipvd_tcp_timeout_ms", 3000),
    ],
)
def test_retired_ovd_field_type_error_explains_migration(field, value):
    """Old recipes must identify the replacement capability and explicit session API."""
    with pytest.raises(TypeError) as error:
        PhysXConfig(**{field: value})
    message = str(error.value)
    assert f"PhysXConfig.{field}" in message
    assert "removed in 0.7" in message
    assert "set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)" in message
    assert "before the first PhysX()" in message
    assert "attach" in message
    assert "PhysX.start_recording" in message
    assert "OmniPvdDestination.file" in message and ".tcp" in message
    assert "docs/ovd_api_migration.md" in message


def _base_signature():
    constructor = inspect.signature(PhysXConfig.__init__)
    return constructor.replace(parameters=list(constructor.parameters.values())[1:])


# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-2
def test_config_signature_matches_generated_init_exactly():
    assert inspect.signature(PhysXConfig) == _base_signature()


# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-2
@pytest.mark.skipif(sys.version_info < (3, 14), reason="Requires lazy annotations")
def test_config_subclass_with_lazy_forward_reference():
    from annotationlib import Format, ForwardRef

    @dataclass
    class AppConfig(PhysXConfig):
        peer: Later = None

    assert AppConfig().peer is None
    parameter = inspect.signature(AppConfig, annotation_format=Format.FORWARDREF).parameters["peer"]
    assert isinstance(parameter.annotation, ForwardRef)
    parameter = inspect.signature(AppConfig, annotation_format=Format.STRING).parameters["peer"]
    assert parameter.annotation == "Later"

    class Later:
        pass

    assert inspect.signature(AppConfig).parameters["peer"].annotation is Later


def test_config_constructor_retains_dataclass_signature_and_positional_order():
    """Migration diagnostics must not replace the generated public constructor signature."""
    names = (
        "disable_contact_processing", "collision_cone_custom_geometry",
        "collision_cylinder_custom_geometry", "num_threads", "scene_multi_gpu_mode",
        "profiler_backend", "profiler_output_path", "cooked_collider_cache_dir",
        "ovstage_read_pool_max_mb", "carbonite_overrides",
    )
    for constructor in (PhysXConfig, PhysXConfig.__init__):
        parameters = list(inspect.signature(constructor).parameters.values())
        if constructor is PhysXConfig.__init__:
            parameters = parameters[1:]
        assert tuple(parameter.name for parameter in parameters) == names
        assert all(parameter.kind == inspect.Parameter.POSITIONAL_OR_KEYWORD for parameter in parameters)
        assert all(parameter.default is None for parameter in parameters)
    cfg = PhysXConfig(True, False, True, 4, 0, "trace", "profile.json", "cache", 256, {"/custom/flag": True})
    assert (cfg.disable_contact_processing, cfg.collision_cone_custom_geometry,
            cfg.collision_cylinder_custom_geometry, cfg.num_threads, cfg.scene_multi_gpu_mode,
            cfg.profiler_backend, cfg.profiler_output_path, cfg.cooked_collider_cache_dir,
            cfg.ovstage_read_pool_max_mb, cfg.carbonite_overrides) == (
                True, False, True, 4, 0, ProfilerBackend.TRACE, "profile.json", "cache", 256, {"/custom/flag": True})
    assert len(_to_c_config(cfg)) == 10


# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-2
def test_config_subclass_signature_includes_added_fields():
    @dataclass
    class AppConfig(PhysXConfig):
        label: str = "default"

    class LocalConfig(AppConfig):
        pass

    for constructor in (AppConfig, LocalConfig):
        inspect.signature(constructor).bind(label="custom")
        assert list(inspect.signature(constructor).parameters)[-1] == "label"
        assert constructor(label="custom").label == "custom"


# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-2
@pytest.mark.parametrize("metaclass", [type, abc.ABCMeta])
def test_config_plain_and_abc_subclass_signature_matches_base(metaclass):
    class AppConfig(PhysXConfig, metaclass=metaclass):
        pass

    assert inspect.signature(AppConfig) == _base_signature()


# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-2
def test_config_wrapped_factory_preserves_signature():
    @wraps(PhysXConfig)
    def make_config(*args, **kwargs):
        return PhysXConfig(*args, **kwargs)

    constructor = inspect.signature(make_config)
    assert constructor == _base_signature()
    constructor.bind(num_threads=4)
    assert make_config(num_threads=4).num_threads == 4
    # A copied __signature__ must remain usable as a normal inspect.Signature.
    assert constructor.replace(return_annotation=int).return_annotation is int
    for cloned in (copy(constructor), deepcopy(constructor), pickle.loads(pickle.dumps(constructor))):
        assert cloned == constructor


# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-2
def test_config_signature_survives_module_reload():
    # Reload in a child so the package's cached exports and this suite's classes
    # still refer to the same config module generation in the parent process.
    code = textwrap.dedent("""
        import importlib
        import inspect
        import ovphysx.config as config

        old_config = config.PhysXConfig
        class OldSubclass(old_config):
            pass

        expected = inspect.signature(old_config)
        importlib.reload(config)
        assert config.PhysXConfig is not old_config
        for constructor in (old_config, OldSubclass, config.PhysXConfig):
            assert inspect.signature(constructor) == expected
            assert constructor(num_threads=4).num_threads == 4
    """)
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join(sys.path)
    result = subprocess.run([sys.executable, "-c", code], env=env, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr


# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-2
def test_config_mixin_signature_uses_nearest_constructor():
    class Mixin:
        def __init__(self, tag, **kwargs):
            super().__init__(**kwargs)
            self.tag = tag

    class MixedConfig(Mixin, PhysXConfig):
        pass

    assert str(inspect.signature(MixedConfig)) == "(tag, **kwargs)"
    inspect.signature(MixedConfig).bind("custom", num_threads=4)
    cfg = MixedConfig("custom", num_threads=4)
    assert cfg.tag == "custom" and cfg.num_threads == 4


# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-2
def test_config_variadic_constructor_signature_retains_args():
    class VariadicConfig(PhysXConfig):
        def __init__(*args, **kwargs):
            PhysXConfig.__init__(args[0], **kwargs)

    assert str(inspect.signature(VariadicConfig)) == "(*args, **kwargs)"
    inspect.signature(VariadicConfig).bind(num_threads=4)
    assert VariadicConfig(num_threads=4).num_threads == 4


# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-2
def test_config_callable_instance_signature_uses_call():
    class CallableConfig(PhysXConfig):
        def __call__(self, value, /, *, scale=1):
            return value * scale

    cfg = CallableConfig()
    assert str(inspect.signature(cfg)) == "(value, /, *, scale=1)"
    inspect.signature(cfg).bind(3, scale=2)
    assert cfg(3, scale=2) == 6


# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-2
def test_config_metaclass_signature_uses_call():
    class ConfigMeta(type):
        def __call__(cls, tag, **kwargs):
            cfg = super().__call__(**kwargs)
            cfg.tag = tag
            return cfg

    class MetaConfig(PhysXConfig, metaclass=ConfigMeta):
        pass

    assert str(inspect.signature(MetaConfig)) == "(tag, **kwargs)"
    inspect.signature(MetaConfig).bind("custom", num_threads=4)
    cfg = MetaConfig("custom", num_threads=4)
    assert cfg.tag == "custom" and cfg.num_threads == 4


# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-2
def test_config_inherited_metaclass_call_takes_precedence():
    class ConfigMeta(type):
        def __call__(cls, tag, **kwargs):
            return super().__call__(**kwargs)

    class SubMeta(ConfigMeta):
        pass

    class MetaConfig(PhysXConfig, metaclass=SubMeta):
        pass

    assert str(inspect.signature(MetaConfig)) == "(tag, **kwargs)"


# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-2
def test_config_positional_only_self_is_dropped():
    class PosOnlyConfig(PhysXConfig):
        def __init__(self, /, tag="t", **kwargs):
            super().__init__(**kwargs)
            self.tag = tag

    assert str(inspect.signature(PosOnlyConfig)) == "(tag='t', **kwargs)"
    inspect.signature(PosOnlyConfig).bind(tag="custom")
    assert PosOnlyConfig(tag="custom").tag == "custom"


# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-2
def test_config_custom_new_signature_is_not_overridden():
    class NewConfig(PhysXConfig):
        def __new__(cls, tag, /, **kwargs):
            return super().__new__(cls, **kwargs)

        def __init__(self, tag, **kwargs):
            super().__init__(**kwargs)
            self.tag = tag

    class LocalConfig(NewConfig):
        pass

    for constructor in (NewConfig, LocalConfig):
        assert str(inspect.signature(constructor)) == "(tag, /, **kwargs)"
        inspect.signature(constructor).bind("custom", num_threads=4)
        cfg = constructor("custom", num_threads=4)
        assert cfg.tag == "custom" and cfg.num_threads == 4


# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-2
def test_config_subclass_retired_fields_keep_migration_error():
    @dataclass
    class AppConfig(PhysXConfig):
        label: str = "default"

    class LocalConfig(AppConfig):
        pass

    for constructor in (AppConfig, LocalConfig):
        with pytest.raises(TypeError, match="set_global_config"):
            constructor(label="custom", omnipvd_recording_capable=True)


def test_config_retains_replace_deepcopy_and_pickle():
    """The diagnostic guard must leave normal dataclass cloning and serialization intact."""
    cfg = PhysXConfig(num_threads=4, profiler_backend="trace", carbonite_overrides={"/custom/flag": True})
    replacement = replace(cfg, num_threads=2)
    assert replacement.num_threads == 2 and cfg.num_threads == 4
    assert replacement.profiler_backend == ProfilerBackend.TRACE
    for cloned in (deepcopy(cfg), pickle.loads(pickle.dumps(cfg))):
        assert cloned == cfg and cloned is not cfg
        assert cloned.carbonite_overrides is not cfg.carbonite_overrides
        assert len(_to_c_config(cloned)) == 3


def test_config_unknown_keyword_still_raises_standard_type_error():
    with pytest.raises(TypeError, match="unexpected keyword argument 'not_a_config_field'"):
        PhysXConfig(not_a_config_field=True)


# ---------------------------------------------------------------------------
# Bool fields
# ---------------------------------------------------------------------------

_BOOL_FIELDS = [
    "disable_contact_processing",
    "collision_cone_custom_geometry",
    "collision_cylinder_custom_geometry",
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
# Profiler backend
#
# The only field that takes either a name or an enum value, because the name is how
# the OVPHYSX_PROFILER environment variable spells it. Both normalize to the int the
# C enum expects, so _to_c_config stays a plain lookup.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "value,expected",
    [
        ("none", ProfilerBackend.NONE),
        ("nvtx", ProfilerBackend.NVTX),
        ("trace", ProfilerBackend.TRACE),
        ("NVTX", ProfilerBackend.NVTX),
        ("Trace", ProfilerBackend.TRACE),
        (ProfilerBackend.NVTX, ProfilerBackend.NVTX),
        (2, ProfilerBackend.TRACE),
    ],
)
def test_profiler_backend_normalizes(value, expected):
    """Names are case-insensitive and enum values pass through, both landing on the int."""
    cfg = PhysXConfig(profiler_backend=value)
    assert cfg.profiler_backend == expected


@pytest.mark.parametrize("bad_value", ["tracy", "nsight", ""])
def test_profiler_backend_unknown_name_raises(bad_value):
    """An unrecognized backend name must raise rather than quietly disable profiling."""
    with pytest.raises(ValueError):
        PhysXConfig(profiler_backend=bad_value)


@pytest.mark.parametrize("bad_value", [3, -1, 99])
def test_profiler_backend_out_of_range_raises(bad_value):
    """An int outside the enum must raise."""
    with pytest.raises(ValueError):
        PhysXConfig(profiler_backend=bad_value)


@pytest.mark.parametrize("bad_value", [True, False, 1.0, [], {}])
def test_profiler_backend_wrong_type_raises(bad_value):
    """bool is an int subclass, so it has to be rejected explicitly."""
    with pytest.raises(TypeError):
        PhysXConfig(profiler_backend=bad_value)


# ---------------------------------------------------------------------------
# Int fields
# ---------------------------------------------------------------------------

_INT_FIELDS = ["num_threads", "scene_multi_gpu_mode", "ovstage_read_pool_max_mb"]


@pytest.mark.parametrize(
    "field,value",
    [
        ("num_threads", 1),
        ("num_threads", 4),
        ("num_threads", 0),
        ("scene_multi_gpu_mode", 0),
        ("scene_multi_gpu_mode", 1),
        ("scene_multi_gpu_mode", 2),
        ("ovstage_read_pool_max_mb", 0),
        ("ovstage_read_pool_max_mb", 256),
        # The pool budget accepts the full int32 range: <= 0 disables the pool and INT32_MAX is the
        # largest retention budget. Values outside int32 are not asserted here because the field has
        # no range guard and the C ABI's int32 wraps them silently.
        ("ovstage_read_pool_max_mb", -1),
        ("ovstage_read_pool_max_mb", 2**31 - 1),
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
    """The cooked collider cache directory accepts any string."""
    cfg = PhysXConfig(cooked_collider_cache_dir="/tmp/cache")
    assert cfg.cooked_collider_cache_dir == "/tmp/cache"


def test_str_field_accepts_empty_string():
    """Empty string is a valid directory value."""
    cfg = PhysXConfig(cooked_collider_cache_dir="")
    assert cfg.cooked_collider_cache_dir == ""


@pytest.mark.parametrize("bad_value", [42, True, 3.14, ["/tmp"]])
def test_str_field_rejects_non_str(bad_value):
    """Non-string values for str fields must raise TypeError."""
    with pytest.raises(TypeError):
        PhysXConfig(cooked_collider_cache_dir=bad_value)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"tcp_address": 1},
        {"tcp_port": True},
        {"tcp_timeout_ms": 1.5},
        {"file_path": 1},
    ],
)
def test_ovd_destination_rejects_wrong_types(kwargs):
    with pytest.raises(TypeError):
        OmniPvdDestination(transport="tcp", **kwargs)


@pytest.mark.parametrize("transport", ["", "TCP", "unknown"])
def test_ovd_destination_rejects_noncanonical_transport(transport):
    with pytest.raises(ValueError, match="transport"):
        OmniPvdDestination(transport=transport)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"tcp_port": 5425},
        {"tcp_address": "127.0.0.1"},
        {"tcp_address": "", "tcp_port": 5425},
        {"tcp_address": "127.0.0.1", "tcp_port": 0},
        {"tcp_address": "127.0.0.1", "tcp_port": 65536},
        {
            "tcp_address": "127.0.0.1",
            "tcp_port": 5425,
            "tcp_timeout_ms": -1,
        },
        {
            "tcp_address": "127.0.0.1",
            "tcp_port": 5425,
            "tcp_timeout_ms": 2**31,
        },
    ],
)
def test_ovd_tcp_destination_validation(kwargs):
    with pytest.raises(ValueError):
        OmniPvdDestination(transport="tcp", **kwargs)


def test_ovd_tcp_destination_accepts_complete_tuple():
    destination = OmniPvdDestination.tcp("127.0.0.1", 5425, timeout_ms=3000)
    assert (destination.transport, destination.tcp_address, destination.tcp_port,
            destination.tcp_timeout_ms, destination.file_path) == ("tcp", "127.0.0.1", 5425, 3000, "")


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


# Carbonite paths mapped to typed fields. Duplicating them in
# carbonite_overrides must raise ValueError at _to_c_config time.
_CONFLICTING_CARBONITE_PATHS = [
    "/physics/disableContactProcessing",
    "/physics/collisionApproximateCones",
    "/physics/collisionApproximateCylinders",
    "/persistent/physics/numThreads",
    "/physics/sceneMultiGPUMode",
    "/physics/profilerBackend",
    "/physics/profilerOutputPath",
    "/physics/ovstageReadPoolMaxMB",
]


@pytest.mark.parametrize("path", _CONFLICTING_CARBONITE_PATHS)
def test_carbonite_overrides_conflicts_typed_path(path):
    """A carbonite_overrides key that matches a typed field's Carbonite path must raise ValueError."""
    cfg = PhysXConfig(carbonite_overrides={path: "true"})
    with pytest.raises(ValueError, match="conflicts with typed field"):
        _to_c_config(cfg)


def test_carbonite_overrides_value_coercion():
    """carbonite_overrides values are coerced: bool to 'true'/'false', int to str."""
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
    # No exception means the coercion succeeded.


def test_combined_typed_and_overrides():
    """Typed fields and non-conflicting carbonite_overrides can coexist."""
    cfg = PhysXConfig(
        num_threads=4,
        carbonite_overrides={"/custom/setting": "value"},
    )
    entries = _to_c_config(cfg)
    assert len(entries) == 2  # one typed + one carbonite
