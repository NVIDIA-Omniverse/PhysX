# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-OVSTAGE-SCHEMA-001
# @covers AC-1 AC-3

"""``import ovphysx`` must not touch USD: fresh subprocess per scenario.

The application supplies PhysX schema data and owns registration with its USD
runtime. The ovphysx package therefore must not:

  1. publish or modify the USD plugin-path environment variables
     (``OV_PXR_PLUGINPATH_2511`` for the namespaced runtime ovstage uses,
     ``PXR_PLUGINPATH_NAME`` for a stock OpenUSD) at import time;
  2. load the native library or any USD module at import time.

The application metadata-discovery recipe ``physx_schema_paths()`` must also
leave USD and native bootstrap untouched.

Each scenario runs in its own ``subprocess.run`` so prior test imports (or
pytest's own startup) cannot pollute the environment being measured.
"""

import os
import subprocess
import sys

_NAMESPACED_ENV_VAR = "OV_PXR_PLUGINPATH_2511"
_CLASSIC_ENV_VAR = "PXR_PLUGINPATH_NAME"
_SENTINEL = "/some/external/usd/plugins/path"


def _run_probe(script: str, *, extra_env=None, timeout: int = 60) -> subprocess.CompletedProcess:
    env = os.environ.copy()
    env.pop(_NAMESPACED_ENV_VAR, None)
    env.pop(_CLASSIC_ENV_VAR, None)
    if extra_env:
        env.update(extra_env)
    return subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        env=env,
        timeout=timeout,
    )


def _emit_lines(stdout: str, prefixes):
    out = {}
    for line in stdout.splitlines():
        for prefix in prefixes:
            if line.startswith(prefix):
                out[prefix] = line[len(prefix):]
    return out


_PROBE = r"""
import os
import sys

for _mod in list(sys.modules):
    if _mod == "ovphysx" or _mod.startswith("ovphysx."):
        del sys.modules[_mod]

import ovphysx
from schema_support import physx_schema_paths
print("NAMESPACED_ENV:" + os.environ.get("OV_PXR_PLUGINPATH_2511", "<unset>"))
print("CLASSIC_ENV:" + os.environ.get("PXR_PLUGINPATH_NAME", "<unset>"))
print("BOOTSTRAPPED:" + str(ovphysx._native_bootstrapped))
print("PXR_LOADED:" + str("pxr" in sys.modules))
print("SCHEMA_ROOT:" + str(physx_schema_paths()[0]))
print("BOOTSTRAPPED_AFTER_QUERY:" + str(ovphysx._native_bootstrapped))
print("NAMESPACED_ENV_AFTER_QUERY:" + os.environ.get("OV_PXR_PLUGINPATH_2511", "<unset>"))
print("CLASSIC_ENV_AFTER_QUERY:" + os.environ.get("PXR_PLUGINPATH_NAME", "<unset>"))
print("PXR_LOADED_AFTER_QUERY:" + str("pxr" in sys.modules))
"""

_PREFIXES = (
    "NAMESPACED_ENV:",
    "CLASSIC_ENV:",
    "BOOTSTRAPPED:",
    "PXR_LOADED:",
    "SCHEMA_ROOT:",
    "BOOTSTRAPPED_AFTER_QUERY:",
    "NAMESPACED_ENV_AFTER_QUERY:",
    "CLASSIC_ENV_AFTER_QUERY:",
    "PXR_LOADED_AFTER_QUERY:",
)


def _probe_lines(extra_env=None):
    result = _run_probe(_PROBE, extra_env=extra_env)
    assert result.returncode == 0, (
        f"subprocess probe failed (rc={result.returncode}):\n" f"stdout={result.stdout!r}\nstderr={result.stderr!r}"
    )
    lines = _emit_lines(result.stdout, _PREFIXES)
    missing = [prefix for prefix in _PREFIXES if prefix not in lines]
    assert not missing, f"probe did not emit {missing}:\n{result.stdout}"
    return lines


def test_import_leaves_usd_plugin_paths_unset():
    """No USD plugin-path variable appears as a side effect of ``import ovphysx``."""
    lines = _probe_lines()
    assert lines["NAMESPACED_ENV:"] == "<unset>", f"import ovphysx published {_NAMESPACED_ENV_VAR}: {lines['NAMESPACED_ENV:']!r}"
    assert lines["CLASSIC_ENV:"] == "<unset>", f"import ovphysx published {_CLASSIC_ENV_VAR}: {lines['CLASSIC_ENV:']!r}"
    assert lines["NAMESPACED_ENV_AFTER_QUERY:"] == "<unset>", (
        f"physx_schema_paths() published {_NAMESPACED_ENV_VAR}: {lines['NAMESPACED_ENV_AFTER_QUERY:']!r}"
    )
    assert lines["CLASSIC_ENV_AFTER_QUERY:"] == "<unset>", (
        f"physx_schema_paths() published {_CLASSIC_ENV_VAR}: {lines['CLASSIC_ENV_AFTER_QUERY:']!r}"
    )


def test_import_preserves_application_owned_plugin_paths():
    """Variables the application set before Python started come through unchanged."""
    lines = _probe_lines(extra_env={_NAMESPACED_ENV_VAR: _SENTINEL, _CLASSIC_ENV_VAR: _SENTINEL})
    assert lines["NAMESPACED_ENV:"] == _SENTINEL, f"import ovphysx modified {_NAMESPACED_ENV_VAR}: {lines['NAMESPACED_ENV:']!r}"
    assert lines["CLASSIC_ENV:"] == _SENTINEL, f"import ovphysx modified {_CLASSIC_ENV_VAR}: {lines['CLASSIC_ENV:']!r}"
    assert lines["NAMESPACED_ENV_AFTER_QUERY:"] == _SENTINEL, (
        f"physx_schema_paths() modified {_NAMESPACED_ENV_VAR}: {lines['NAMESPACED_ENV_AFTER_QUERY:']!r}"
    )
    assert lines["CLASSIC_ENV_AFTER_QUERY:"] == _SENTINEL, (
        f"physx_schema_paths() modified {_CLASSIC_ENV_VAR}: {lines['CLASSIC_ENV_AFTER_QUERY:']!r}"
    )


def test_import_and_schema_query_load_no_native_or_usd_code():
    """Import and schema discovery stay pure Python: no native bootstrap, no pxr."""
    lines = _probe_lines()
    assert lines["BOOTSTRAPPED:"] == "False", f"import ovphysx triggered native load: {lines['BOOTSTRAPPED:']!r}"
    assert lines["BOOTSTRAPPED_AFTER_QUERY:"] == "False", (
        f"physx_schema_paths() triggered native load: {lines['BOOTSTRAPPED_AFTER_QUERY:']!r}"
    )
    assert lines["PXR_LOADED:"] == "False", "import ovphysx imported pxr"
    assert lines["PXR_LOADED_AFTER_QUERY:"] == "False", "physx_schema_paths() imported pxr"


def test_schema_root_points_at_registrable_codeless_schemas():
    """Metadata discovery finds registrable resources in the external provider."""
    lines = _probe_lines()
    root = lines["SCHEMA_ROOT:"]
    assert os.path.isdir(root), f"physx_schema_paths() is not a directory: {root!r}"
    assert os.path.isfile(os.path.join(root, "plugInfo.json")), f"missing root plugInfo.json under {root!r}"
    assert os.path.isfile(os.path.join(root, "generatedSchema.usda"))
