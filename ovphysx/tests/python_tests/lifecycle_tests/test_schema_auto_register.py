# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Auto-register schema-paths tests — fresh subprocess per scenario.

Validates the public contract for ``register_schema_paths()`` and the
``import ovphysx`` side effect on ``OV_PXR_PLUGINPATH_2511``:

  1. ``import ovphysx`` appends ovphysx's ``plugins/usd`` to the env var
     without triggering native library loading (USD and native bindings stay deferred).
  2. If the env var was set BEFORE Python startup (i.e. by the parent
     process or by C-level ``setenv`` in a co-tenant library), the existing
     value is preserved and ovphysx's path is appended — never replaced.
  3. ``register_schema_paths()`` is always-idempotent: callers may pop the
     env var from ``os.environ`` and re-call it to restore registration.
     Repeated calls without intervening pops are cheap no-ops.

Each scenario runs in its own ``subprocess.run`` so prior test imports (or
pytest's own startup) cannot pollute the env-var or ``_native_bootstrapped``
state being measured.
"""

import os
import subprocess
import sys

_ENV_VAR = "OV_PXR_PLUGINPATH_2511"


def _run_probe(script: str, *, extra_env=None, timeout: int = 60) -> subprocess.CompletedProcess:
    """Run ``script`` in a fresh interpreter and return the completed process.

    ``extra_env`` lets the caller pre-set env vars that the spawned interpreter
    will inherit at startup (the moral equivalent of a co-tenant library
    calling ``setenv()`` before Python imports ovphysx).
    """
    env = os.environ.copy()
    # Always strip the var from the parent env so the test starts from a
    # known baseline; callers that need it pre-set re-add via extra_env.
    env.pop(_ENV_VAR, None)
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
    """Pull ``KEY:value`` lines out of probe stdout, keyed by prefix."""
    out = {}
    for line in stdout.splitlines():
        for prefix in prefixes:
            if line.startswith(prefix):
                out[prefix] = line[len(prefix) :]
    return out


_PROBE_IMPORT_ONLY = r"""
import os
import sys

for _mod in list(sys.modules):
    if _mod == "ovphysx" or _mod.startswith("ovphysx."):
        del sys.modules[_mod]

import ovphysx

print("ENV:" + os.environ.get("OV_PXR_PLUGINPATH_2511", ""))
print("BOOTSTRAPPED:" + str(ovphysx._native_bootstrapped))
"""


def test_import_appends_schema_path_without_native_load():
    """``import ovphysx`` publishes the env var and stays deferred from native load.

    Two assertions, both load-bearing:
      1. The env var contains a path that points at an existing
         ``plugins/usd`` directory — i.e. registration ran and found the
         shipped ovphysx schema root.
      2. ``ovphysx._native_bootstrapped`` is still False after the import —
         registration is pure-Python and must not pull in USD or native bindings.
    """
    result = _run_probe(_PROBE_IMPORT_ONLY)
    assert result.returncode == 0, (
        f"subprocess probe failed (rc={result.returncode}):\n" f"stdout={result.stdout!r}\nstderr={result.stderr!r}"
    )

    lines = _emit_lines(result.stdout, ("ENV:", "BOOTSTRAPPED:"))
    assert "ENV:" in lines, f"probe did not emit ENV line:\n{result.stdout}"
    assert "BOOTSTRAPPED:" in lines, f"probe did not emit BOOTSTRAPPED line:\n{result.stdout}"

    env_value = lines["ENV:"]
    bootstrapped = lines["BOOTSTRAPPED:"].strip()

    assert bootstrapped == "False", f"import ovphysx triggered native load: _native_bootstrapped={bootstrapped!r}"
    assert env_value, "OV_PXR_PLUGINPATH_2511 was not set after import ovphysx"
    paths = [p for p in env_value.split(os.pathsep) if p]
    matching = [p for p in paths if os.path.isdir(p) and p.replace("\\", "/").endswith("/plugins/usd")]
    assert matching, (
        f"OV_PXR_PLUGINPATH_2511 does not contain an existing plugins/usd directory.\n"
        f"  value={env_value!r}\n"
        f"  parsed paths={paths!r}"
    )


def test_import_preserves_pre_set_env_var():
    """A pre-existing env var (e.g. set by a co-tenant native library before
    Python startup) must be preserved; ovphysx appends rather than replaces.

    This exercises the ``_native_getenv`` merge path: the parent-process env
    populates ``os.environ`` at interpreter startup, but the equivalent live
    C env value is what a native co-tenant would have written. We verify that
    when both views agree, the pre-existing entry survives the merge and
    ovphysx's path is appended after it.
    """
    sentinel = "/some/external/usd/plugins/path/from/cotenant"
    result = _run_probe(_PROBE_IMPORT_ONLY, extra_env={_ENV_VAR: sentinel})
    assert result.returncode == 0, (
        f"subprocess probe failed (rc={result.returncode}):\n" f"stdout={result.stdout!r}\nstderr={result.stderr!r}"
    )

    lines = _emit_lines(result.stdout, ("ENV:",))
    env_value = lines.get("ENV:", "")
    assert env_value, "OV_PXR_PLUGINPATH_2511 was not set after import ovphysx"
    paths = [p for p in env_value.split(os.pathsep) if p]

    assert sentinel in paths, (
        f"pre-set env var entry was dropped during ovphysx auto-register.\n"
        f"  value={env_value!r}\n  parsed paths={paths!r}"
    )
    matching = [p for p in paths if os.path.isdir(p) and p.replace("\\", "/").endswith("/plugins/usd")]
    assert matching, (
        f"ovphysx plugins/usd was not appended to the pre-set env var.\n"
        f"  value={env_value!r}\n  parsed paths={paths!r}"
    )
    # And the sentinel must come BEFORE the appended ovphysx entry — append,
    # not prepend, so existing search order is preserved.
    sentinel_idx = paths.index(sentinel)
    ovphysx_idx = min(paths.index(p) for p in matching)
    assert sentinel_idx < ovphysx_idx, (
        f"ovphysx auto-register prepended instead of appended.\n" f"  parsed paths={paths!r}"
    )


_PROBE_REREGISTER = r"""
import os
import sys

for _mod in list(sys.modules):
    if _mod == "ovphysx" or _mod.startswith("ovphysx."):
        del sys.modules[_mod]

import ovphysx

# Capture state after auto-register-on-import.
print("AFTER_IMPORT:" + os.environ.get("OV_PXR_PLUGINPATH_2511", ""))

# Pop the env var entirely and re-register: the always-idempotent contract
# requires register_schema_paths() to restore the ovphysx path even after
# the var was wiped. (No sticky guard prevents re-registration.)
os.environ.pop("OV_PXR_PLUGINPATH_2511", None)
print("AFTER_POP:" + os.environ.get("OV_PXR_PLUGINPATH_2511", ""))

ovphysx.register_schema_paths()
print("AFTER_REREGISTER:" + os.environ.get("OV_PXR_PLUGINPATH_2511", ""))

# Re-call without popping: should be a cheap no-op that produces the same
# value (idempotent under repeated invocation).
ovphysx.register_schema_paths()
print("AFTER_DOUBLE:" + os.environ.get("OV_PXR_PLUGINPATH_2511", ""))
"""


def test_register_schema_paths_is_always_idempotent():
    """``register_schema_paths()`` must be re-callable after env-var pop.

    Locks in the always-idempotent contract:
      1. After ``import ovphysx``, the env var contains ovphysx's plugins/usd.
      2. After ``os.environ.pop(...)``, the env var is empty.
      3. After ``register_schema_paths()``, the env var is restored — the
         function is NOT gated by a sticky ``_registration_done`` flag.
      4. A second back-to-back call produces the same value (idempotent).
    """
    result = _run_probe(_PROBE_REREGISTER)
    assert result.returncode == 0, (
        f"subprocess probe failed (rc={result.returncode}):\n" f"stdout={result.stdout!r}\nstderr={result.stderr!r}"
    )

    lines = _emit_lines(
        result.stdout,
        ("AFTER_IMPORT:", "AFTER_POP:", "AFTER_REREGISTER:", "AFTER_DOUBLE:"),
    )
    after_import = lines.get("AFTER_IMPORT:", "")
    after_pop = lines.get("AFTER_POP:", "")
    after_reregister = lines.get("AFTER_REREGISTER:", "")
    after_double = lines.get("AFTER_DOUBLE:", "")

    assert after_import, f"env var was empty after import:\n{result.stdout}"
    assert after_pop == "", f"os.environ.pop did not clear the env var: {after_pop!r}"
    assert after_reregister, (
        f"register_schema_paths() failed to restore the env var after pop — "
        f"sticky-guard regression?\n{result.stdout}"
    )

    # Re-registration must produce a value containing an existing plugins/usd
    # directory under ovphysx (same contract as the import-time check).
    paths = [p for p in after_reregister.split(os.pathsep) if p]
    matching = [p for p in paths if os.path.isdir(p) and p.replace("\\", "/").endswith("/plugins/usd")]
    assert matching, (
        f"register_schema_paths() did not restore an existing plugins/usd entry.\n"
        f"  value={after_reregister!r}\n  parsed paths={paths!r}"
    )

    # Second call must be a stable no-op.
    assert after_double == after_reregister, (
        f"register_schema_paths() is not idempotent under repeated calls.\n"
        f"  after_reregister={after_reregister!r}\n  after_double={after_double!r}"
    )
