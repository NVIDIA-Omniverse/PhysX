# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""
Namespaced USD plugin-path setup for ovphysx.

ovphysx ships its own namespaced USD runtime. If the host Python process has
already imported ``pxr``, this module does not validate or modify that host USD
runtime. It only exposes ovphysx's namespaced schema plugin path through
``OV_PXR_PLUGINPATH_2511`` so the native bootstrap can keep the two USD runtimes
isolated.
"""

import ctypes
import logging
import os
import sys
import threading
from pathlib import Path
from typing import Optional

_logger = logging.getLogger(__name__)
_NAMESPACED_USD_PLUGIN_PATH_ENV_VAR = "OV_PXR_PLUGINPATH_2511"
_registration_lock = threading.Lock()


def _native_getenv(name: str) -> str:
    """Read an env var via the OS-level getenv, which sees writes by native
    code that don't propagate to Python's ``os.environ`` cache.

    CPython's ``os.environ`` is a Python-side dict that's initialized once at
    interpreter startup from the inherited environment, then maintained by
    Python writes (``os.environ[X] = Y`` calls native ``setenv`` under the
    hood, so Python writes propagate both ways). But native code that calls
    ``setenv()`` / ``_putenv_s()`` directly updates the live C / process
    environment WITHOUT touching Python's dict cache -- so a subsequent
    ``os.environ.get(X)`` returns ``None`` while the live env actually has
    a value. This helper reads through to that live value.
    """
    if sys.platform == "win32":
        # GetEnvironmentVariableW reflects writes by SetEnvironmentVariable
        # AND CRT _putenv_s (the CRT keeps them in sync on Windows).
        kernel32 = ctypes.windll.kernel32
        size = kernel32.GetEnvironmentVariableW(name, None, 0)
        if size == 0:
            return ""
        buf = ctypes.create_unicode_buffer(size)
        n = kernel32.GetEnvironmentVariableW(name, buf, size)
        return buf.value if n > 0 else ""
    libc = ctypes.CDLL(None)
    libc.getenv.restype = ctypes.c_char_p
    libc.getenv.argtypes = [ctypes.c_char_p]
    raw = libc.getenv(name.encode("utf-8"))
    return raw.decode("utf-8") if raw else ""


def _normalize_path(path: Path) -> str:
    try:
        return str(path.resolve(strict=False))
    except OSError:
        return str(path)


def _candidate_plugin_paths():
    candidates = []
    lib_path = os.environ.get("OVPHYSX_LIB")
    if lib_path:
        try:
            override_path = Path(lib_path).resolve(strict=False)
            lib_dir = override_path if override_path.is_dir() else override_path.parent
            derived_paths = [
                lib_dir.parent / "plugins" / "usd",
                lib_dir / "plugins" / "usd",
            ]
            for derived in derived_paths:
                if derived not in candidates:
                    candidates.append(derived)
            _logger.info(
                "Deriving USD plugin paths from OVPHYSX_LIB: %s",
                ", ".join(str(path) for path in derived_paths),
            )
        except OSError:
            pass

    module_dir = Path(__file__).parent
    candidates.append(module_dir / "plugins" / "usd")
    return candidates


def _ensure_pxr_plugin_path() -> Optional[str]:
    """Ensure the namespaced USD plugin-path env var is set for this ovphysx package.

    Reads from BOTH ``os.environ`` (Python-side writes) and the live C env
    via ``_native_getenv`` (writes by native ``setenv``/``_putenv_s`` that
    don't reach Python's dict cache), merges preserving order, dedupes,
    appends the ovphysx ``plugins/usd`` if missing, and writes the result
    back via ``os.environ`` (which propagates to the live C env on its
    own). Always-idempotent: repeated calls produce the same final state.
    """
    existing_candidates = [candidate for candidate in _candidate_plugin_paths() if candidate.is_dir()]
    if not existing_candidates:
        return None

    py_existing = os.environ.get(_NAMESPACED_USD_PLUGIN_PATH_ENV_VAR, "")
    native_existing = _native_getenv(_NAMESPACED_USD_PLUGIN_PATH_ENV_VAR)

    # Merge native + python views, preserving insertion order, deduping by
    # raw string (we de-dup again below by canonical path so an entry that
    # appears in both forms collapses too).
    paths: list = []
    seen: set = set()
    for source in (native_existing, py_existing):
        for entry in source.split(os.pathsep):
            if entry and entry not in seen:
                paths.append(entry)
                seen.add(entry)

    normalized_paths = {_normalize_path(Path(p)) for p in paths}
    for candidate in existing_candidates:
        candidate_str = _normalize_path(candidate)
        if candidate_str not in normalized_paths:
            paths.append(candidate_str)
            normalized_paths.add(candidate_str)

    new_value = os.pathsep.join(paths)
    # Write back if either: (a) we appended ovphysx, or (b) the native env
    # had something Python didn't (so the merge produced a value Python's
    # dict doesn't know about yet).
    if new_value != py_existing:
        os.environ[_NAMESPACED_USD_PLUGIN_PATH_ENV_VAR] = new_value
    return _NAMESPACED_USD_PLUGIN_PATH_ENV_VAR


def register_schema_paths() -> None:
    """Register ovphysx's namespaced USD schema/plugin paths before native bootstrap.

    Call this before any USD stage open or schema-registry access when ovphysx
    shares a process with another USD-aware subsystem such as ovrtx. Standalone
    ovphysx applications do not need to call it because native startup registers
    the same ovphysx path automatically.

    Always idempotent: re-reads the live env (Python and native views), merges,
    dedupes, and writes back only if the merged value differs. Safe to call
    repeatedly and after callers pop ``OV_PXR_PLUGINPATH_2511`` from
    ``os.environ`` to force re-registration.

    Raises:
        RuntimeError: If no existing ovphysx ``plugins/usd`` directory can be found.
    """
    with _registration_lock:
        env_var = _ensure_pxr_plugin_path()
        if env_var is None:
            candidates = ", ".join(str(path) for path in _candidate_plugin_paths()) or "<none>"
            raise RuntimeError(
                "Failed to register ovphysx USD schema/plugin paths: no existing "
                f"plugins/usd directory found. Checked: {candidates}. Set OVPHYSX_LIB "
                "to the ovphysx shared library path or reinstall the ovphysx package."
            )


def check_usd_compatibility():
    """
    Set ovphysx's namespaced plugin path if Python already loaded pxr.

    A host ``pxr`` module may be a classic USD runtime and does not need to
    match ovphysx's bundled namespaced USD. Do not touch ``PXR_PLUGINPATH_NAME``
    or reload the host registry here.
    """
    if "pxr" not in sys.modules:
        return

    _ensure_pxr_plugin_path()
    _logger.info(
        "Host pxr is already loaded; ovphysx uses its bundled namespaced USD, "
        "so host USD version and schema validation are skipped."
    )
